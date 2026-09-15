#include "rm_image_compression/temporal_frame_composer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace rm_image_compression
{

TemporalFrameComposer::TemporalFrameComposer(
  const double output_fps,
  CodecConfig::TemporalAggregationConfig config)
: output_fps_(std::max(output_fps, 1.0)),
  frame_period_sec_(1.0 / output_fps_),
  trail_blend_alpha_(std::clamp(config.trail_blend_alpha, 0.0, 1.0)),
  static_simplify_(config.static_simplify),
  motion_threshold_(std::max(config.motion_threshold, 0)),
  motion_erode_px_(std::clamp(config.motion_erode_px, 0, 20)),
  motion_dilate_px_(std::clamp(config.motion_dilate_px, 0, 20)),
  motion_trail_frames_(std::clamp(config.motion_trail_frames, 0, 15)),
  trail_disable_motion_ratio_(std::clamp(config.trail_disable_motion_ratio, 0.0, 1.0)),
  bg_update_alpha_(std::clamp(config.bg_update_alpha, 0.001, 0.2)),
  bg_blur_sigma_(std::max(config.bg_blur_sigma, 0.0)),
  center_clear_size_(std::max(config.center_clear_size, 0)),
  center_color_roi_(config.center_color_roi),
  active_(true)
{
}

std::optional<TemporalCompositionResult> TemporalFrameComposer::push_frame(
  const cv::Mat & bgr_frame,
  const std::int64_t source_timestamp_ns)
{
  if (bgr_frame.empty()) {
    throw std::invalid_argument("TemporalFrameComposer received an empty frame");
  }
  if (bgr_frame.type() != CV_8UC3) {
    throw std::invalid_argument("TemporalFrameComposer expects CV_8UC3 input");
  }

  TemporalCompositionResult processed = preprocess_frame(bgr_frame);
  if (!active_) {
    processed.source_frames_aggregated = 1;
    return processed;
  }

  latest_output_frame_ = std::move(processed.frame_bgr);
  latest_used_high_fps_trail_ = processed.used_high_fps_trail;
  buffered_frame_count_ += 1;

  const double source_time_sec =
    static_cast<double>(source_timestamp_ns) / 1000000000.0;
  if (!should_emit(source_time_sec)) {
    return std::nullopt;
  }

  TemporalCompositionResult result = compose_pending_output();
  reset_pending_output();
  return result;
}

std::optional<TemporalCompositionResult> TemporalFrameComposer::flush()
{
  if (!active_ || buffered_frame_count_ <= 0 || latest_output_frame_.empty()) {
    return std::nullopt;
  }
  TemporalCompositionResult result = compose_pending_output();
  reset_pending_output();
  return result;
}

bool TemporalFrameComposer::active() const noexcept
{
  return active_;
}

int TemporalFrameComposer::buffered_frame_count() const noexcept
{
  return buffered_frame_count_;
}

bool TemporalFrameComposer::should_emit(const double source_time_sec)
{
  if (!selector_initialized_) {
    selector_initialized_ = true;
    // Source timestamps are absolute epoch-based seconds. Seeding from 0 would
    // require billions of loop iterations on the first frame.
    next_emit_time_sec_ = source_time_sec + frame_period_sec_;
    return true;
  }
  if (source_time_sec + 1e-9 < next_emit_time_sec_) {
    return false;
  }
  do {
    next_emit_time_sec_ += frame_period_sec_;
  } while (next_emit_time_sec_ <= source_time_sec + 1e-9);
  return true;
}

void TemporalFrameComposer::apply_center_color_roi(cv::Mat * frame_bgr) const
{
  if (frame_bgr == nullptr || frame_bgr->empty() || frame_bgr->type() != CV_8UC3) {
    return;
  }

  const int roi_width = std::max(1, frame_bgr->cols / 3);
  const int roi_height = std::max(1, frame_bgr->rows / 3);
  const int x0 = std::max(0, (frame_bgr->cols - roi_width) / 2);
  const int y0 = std::max(0, (frame_bgr->rows - roi_height) / 2);
  const int rw = std::min(roi_width, frame_bgr->cols - x0);
  const int rh = std::min(roi_height, frame_bgr->rows - y0);

  cv::Mat gray;
  cv::cvtColor(*frame_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::Mat gray_bgr;
  cv::cvtColor(gray, gray_bgr, cv::COLOR_GRAY2BGR);

  const cv::Rect roi(x0, y0, rw, rh);
  (*frame_bgr)(roi).copyTo(gray_bgr(roi));
  *frame_bgr = std::move(gray_bgr);
}

TemporalCompositionResult TemporalFrameComposer::preprocess_frame(const cv::Mat & bgr_frame)
{
  TemporalCompositionResult result;

  if (!static_simplify_) {
    result.frame_bgr = bgr_frame.clone();
    result.used_high_fps_trail = false;
    if (center_color_roi_) {
      apply_center_color_roi(&result.frame_bgr);
    }
    return result;
  }

  cv::Mat gray;
  cv::cvtColor(bgr_frame, gray, cv::COLOR_BGR2GRAY);
  if (background_gray_f32_.empty()) {
    gray.convertTo(background_gray_f32_, CV_32F);
    result.frame_bgr = bgr_frame.clone();
    result.used_high_fps_trail = false;
    if (center_color_roi_) {
      apply_center_color_roi(&result.frame_bgr);
    }
    return result;
  }

  cv::Mat bg_u8;
  cv::convertScaleAbs(background_gray_f32_, bg_u8);

  cv::Mat diff;
  cv::absdiff(gray, bg_u8, diff);

  cv::Mat motion_mask;
  cv::threshold(diff, motion_mask, motion_threshold_, 255, cv::THRESH_BINARY);
  if (motion_erode_px_ > 0) {
    if (motion_erode_kernel_.empty()) {
      const int k = 2 * motion_erode_px_ + 1;
      motion_erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::erode(motion_mask, motion_mask, motion_erode_kernel_, cv::Point(-1, -1), 1);
  }
  if (motion_dilate_px_ > 0) {
    if (motion_dilate_kernel_.empty()) {
      const int k = 2 * motion_dilate_px_ + 1;
      motion_dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    }
    cv::dilate(motion_mask, motion_mask, motion_dilate_kernel_, cv::Point(-1, -1), 1);
  }

  //如果画面有大部分面积在动，说明刚射出弹丸，不用背景差分
  const double motion_ratio_raw =
    static_cast<double>(cv::countNonZero(motion_mask)) / static_cast<double>(motion_mask.total());
  const bool suppress_trail = (motion_ratio_raw >= trail_disable_motion_ratio_);

  //画面中心保持最高清晰度
  if (center_clear_size_ > 0) {
    const int clear_size = std::min({center_clear_size_, bgr_frame.cols, bgr_frame.rows});
    const int x0 = std::max(0, bgr_frame.cols / 2 - clear_size / 2);
    const int y0 = std::max(0, bgr_frame.rows / 2 - clear_size / 2);
    const int cw = std::min(clear_size, bgr_frame.cols - x0);
    const int ch = std::min(clear_size, bgr_frame.rows - y0);
    cv::rectangle(motion_mask, cv::Rect(x0, y0, cw, ch), cv::Scalar(255), cv::FILLED);
  }

  cv::Mat blurred_static;
  cv::GaussianBlur(
    bgr_frame,
    blurred_static,
    cv::Size(),
    bg_blur_sigma_,
    bg_blur_sigma_);

  cv::Mat focused = blurred_static.clone();
  bgr_frame.copyTo(focused, motion_mask);

  //获取弹丸轨迹
  if (motion_trail_frames_ > 0) {
    motion_mask_history_.push_back(motion_mask.clone());
    trail_frame_history_.push_back(bgr_frame.clone());
    const std::size_t max_history = static_cast<std::size_t>(motion_trail_frames_ + 1);
    while (motion_mask_history_.size() > max_history) {
      motion_mask_history_.pop_front();
    }
    while (trail_frame_history_.size() > max_history) {
      trail_frame_history_.pop_front();
    }
    const std::size_t history_size = motion_mask_history_.size();
    if (!suppress_trail && history_size > 1 && history_size == trail_frame_history_.size()) {
      cv::Mat trail_mask = motion_mask.clone();
      cv::Mat trail_img = bgr_frame.clone();
      for (std::size_t i = 0; i + 1 < history_size; ++i) {
        cv::bitwise_or(trail_mask, motion_mask_history_[i], trail_mask);
        cv::max(trail_img, trail_frame_history_[i], trail_img);
      }
      trail_img.copyTo(focused, trail_mask);
      result.used_high_fps_trail = true;
    } else {
      result.used_high_fps_trail = false;
    }
  } else {
    motion_mask_history_.clear();
    trail_frame_history_.clear();
    result.used_high_fps_trail = false;
  }

  cv::accumulateWeighted(gray, background_gray_f32_, bg_update_alpha_);
  result.frame_bgr = std::move(focused);
  if (center_color_roi_) {
    apply_center_color_roi(&result.frame_bgr);
  }
  return result;
}

TemporalCompositionResult TemporalFrameComposer::compose_pending_output() const
{
  if (buffered_frame_count_ <= 0 || latest_output_frame_.empty()) {
    throw std::runtime_error("TemporalFrameComposer has no pending output frame");
  }

  TemporalCompositionResult result;
  result.frame_bgr = latest_output_frame_.clone();
  result.source_frames_aggregated = buffered_frame_count_;
  result.used_high_fps_trail = latest_used_high_fps_trail_;
  return result;
}

void TemporalFrameComposer::reset_pending_output()
{
  buffered_frame_count_ = 0;
  latest_used_high_fps_trail_ = false;
  latest_output_frame_.release();
}

}  // namespace rm_image_compression
