#pragma once

#include "rm_image_compression/codec.hpp"

#include <opencv2/core.hpp>

#include <deque>
#include <cstdint>
#include <optional>

namespace rm_image_compression
{

struct TemporalCompositionResult
{
  cv::Mat frame_bgr;
  int source_frames_aggregated = 0;
  bool used_high_fps_trail = false;
};

class TemporalFrameComposer
{
public:
  TemporalFrameComposer(double output_fps, CodecConfig::TemporalAggregationConfig config);

  std::optional<TemporalCompositionResult> push_frame(
    const cv::Mat & bgr_frame,
    std::int64_t source_frame_index);

  std::optional<TemporalCompositionResult> flush();

  bool active() const noexcept;
  int buffered_frame_count() const noexcept;

private:
  bool should_emit(double source_time_sec);
  TemporalCompositionResult preprocess_frame(const cv::Mat & bgr_frame);
  TemporalCompositionResult compose_pending_output() const;
  void reset_pending_output();

  double source_fps_ = 24.0;
  double output_fps_ = 24.0;
  double frame_period_sec_ = 1.0 / 24.0;
  double trail_blend_alpha_ = 1.0;
  bool static_simplify_ = true;
  int motion_threshold_ = 14;
  int motion_erode_px_ = 1;
  int motion_dilate_px_ = 2;
  int motion_trail_frames_ = 3;
  double trail_disable_motion_ratio_ = 0.30;
  double bg_update_alpha_ = 0.01;
  double bg_blur_sigma_ = 1.2;
  int center_clear_size_ = 0;
  bool active_ = false;
  bool selector_initialized_ = false;
  double next_emit_time_sec_ = 0.0;
  int buffered_frame_count_ = 0;
  bool latest_used_high_fps_trail_ = false;
  cv::Mat latest_output_frame_;
  cv::Mat background_gray_f32_;
  cv::Mat motion_erode_kernel_;
  cv::Mat motion_dilate_kernel_;
  std::deque<cv::Mat> motion_mask_history_;
  std::deque<cv::Mat> trail_frame_history_;
};

}  // namespace rm_image_compression
