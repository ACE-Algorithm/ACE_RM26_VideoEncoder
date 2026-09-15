#include "rm_image_compression/receiver_presentation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace rm_image_compression
{
namespace
{

constexpr double kEpsilonSec = 1e-9;

}  // namespace

double quantize_to_display_time_sec(const double ready_time_sec, const double display_fps)
{
  if (display_fps <= 0.0) {
    throw std::invalid_argument("quantize_to_display_time_sec() requires display_fps > 0");
  }
  const double clamped_time = std::max(ready_time_sec, 0.0);
  const double tick_index = std::ceil(clamped_time * display_fps - kEpsilonSec);
  return tick_index / display_fps;
}

double frame_pts_to_time_sec(const std::int64_t pts, const double pts_fps)
{
  if (pts_fps <= 0.0) {
    throw std::invalid_argument("frame_pts_to_time_sec() requires pts_fps > 0");
  }
  return static_cast<double>(pts) / pts_fps;
}

void validate_receiver_presentation_config(const ReceiverPresentationConfig & config)
{
  if (config.display_fps <= 0.0) {
    throw std::invalid_argument("Receiver display_fps must be positive");
  }
  if (config.min_presented_latency_ms < 0.0) {
    throw std::invalid_argument("Receiver min_presented_latency_ms must be non-negative");
  }
}

ReceiverFrameScheduler::ReceiverFrameScheduler(ReceiverPresentationConfig config)
: config_(std::move(config))
{
  validate_receiver_presentation_config(config_);
}

void ReceiverFrameScheduler::push_frame(
  DecodedFrame frame,
  const double source_time_sec,
  const double decode_ready_time_sec)
{
  if (source_time_sec < 0.0) {
    throw std::invalid_argument("push_frame() requires source_time_sec >= 0");
  }
  if (decode_ready_time_sec < 0.0) {
    throw std::invalid_argument("push_frame() requires decode_ready_time_sec >= 0");
  }

  if (
    last_presented_frame_.has_value() &&
    source_time_sec <= last_presented_frame_->source_time_sec + kEpsilonSec)
  {
    return;
  }

  PendingFrame pending;
  pending.frame = std::move(frame);
  pending.source_time_sec = source_time_sec;
  pending.decode_ready_time_sec = decode_ready_time_sec;
  pending.eligible_display_time_sec = std::max(
    decode_ready_time_sec,
    source_time_sec + config_.min_presented_latency_ms / 1000.0);
  pending_frames_.push_back(std::move(pending));
}

std::optional<ReceiverPresentationFrame> ReceiverFrameScheduler::update(const double display_time_sec)
{
  if (display_time_sec < 0.0) {
    throw std::invalid_argument("update() requires display_time_sec >= 0");
  }
  if (has_updated_ && display_time_sec + kEpsilonSec < last_update_time_sec_) {
    throw std::invalid_argument("update() requires non-decreasing display_time_sec");
  }

  has_updated_ = true;
  last_update_time_sec_ = display_time_sec;

  std::optional<ReceiverPresentationFrame> newly_presented;
  while (
    !pending_frames_.empty() &&
    pending_frames_.front().eligible_display_time_sec <= display_time_sec + kEpsilonSec)
  {
    PendingFrame pending = std::move(pending_frames_.front());
    pending_frames_.pop_front();

    ReceiverPresentationFrame presented;
    presented.frame = std::move(pending.frame);
    presented.source_time_sec = pending.source_time_sec;
    presented.decode_ready_time_sec = pending.decode_ready_time_sec;
    presented.eligible_display_time_sec = pending.eligible_display_time_sec;
    presented.display_time_sec = display_time_sec;
    presented.presented_latency_ms =
      (display_time_sec - pending.source_time_sec) * 1000.0;
    last_presented_frame_ = presented;
    newly_presented = last_presented_frame_;
  }

  if (last_presented_frame_.has_value()) {
    last_presented_frame_->display_time_sec = display_time_sec;
    last_presented_frame_->presented_latency_ms =
      (display_time_sec - last_presented_frame_->source_time_sec) * 1000.0;
  }

  return newly_presented;
}

const std::optional<ReceiverPresentationFrame> &
ReceiverFrameScheduler::last_presented_frame() const noexcept
{
  return last_presented_frame_;
}

int ReceiverFrameScheduler::queued_frame_count() const noexcept
{
  return static_cast<int>(pending_frames_.size());
}

void ReceiverFrameScheduler::reset() noexcept
{
  pending_frames_.clear();
  last_presented_frame_.reset();
  last_update_time_sec_ = 0.0;
  has_updated_ = false;
}

const ReceiverPresentationConfig & ReceiverFrameScheduler::config() const noexcept
{
  return config_;
}

}  // namespace rm_image_compression
