#pragma once

#include "rm_image_compression/codec.hpp"

#include <deque>
#include <optional>

namespace rm_image_compression
{

inline constexpr double kDefaultReceiverDisplayFps = 48.0;
inline constexpr double kDefaultReceiverMinPresentedLatencyMs = 400.0;

struct ReceiverPresentationConfig
{
  double display_fps = kDefaultReceiverDisplayFps;
  double min_presented_latency_ms = kDefaultReceiverMinPresentedLatencyMs;
};

struct ReceiverPresentationFrame
{
  DecodedFrame frame;
  double source_time_sec = 0.0;
  double decode_ready_time_sec = 0.0;
  double eligible_display_time_sec = 0.0;
  double display_time_sec = 0.0;
  double presented_latency_ms = 0.0;
};

double quantize_to_display_time_sec(double ready_time_sec, double display_fps);
double frame_pts_to_time_sec(std::int64_t pts, double pts_fps);
void validate_receiver_presentation_config(const ReceiverPresentationConfig & config);

class ReceiverFrameScheduler
{
public:
  explicit ReceiverFrameScheduler(
    ReceiverPresentationConfig config = ReceiverPresentationConfig{});

  void push_frame(
    DecodedFrame frame,
    double source_time_sec,
    double decode_ready_time_sec);

  std::optional<ReceiverPresentationFrame> update(double display_time_sec);

  const std::optional<ReceiverPresentationFrame> & last_presented_frame() const noexcept;
  int queued_frame_count() const noexcept;
  void reset() noexcept;
  const ReceiverPresentationConfig & config() const noexcept;

private:
  struct PendingFrame
  {
    DecodedFrame frame;
    double source_time_sec = 0.0;
    double decode_ready_time_sec = 0.0;
    double eligible_display_time_sec = 0.0;
  };

  ReceiverPresentationConfig config_;
  std::deque<PendingFrame> pending_frames_;
  std::optional<ReceiverPresentationFrame> last_presented_frame_;
  double last_update_time_sec_ = 0.0;
  bool has_updated_ = false;
};

}  // namespace rm_image_compression
