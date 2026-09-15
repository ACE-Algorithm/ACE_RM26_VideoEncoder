#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace rm_image_compression
{

inline constexpr double kMaxSupportedFps = 48.0;
inline constexpr std::size_t kPacketHeaderBytes = 17;
inline constexpr std::size_t kMaxSupportedPacketBytes = 300;

struct CodecConfig
{
  struct TemporalAggregationConfig
  {
    bool enabled = false;
    double source_fps = 0.0;
    double trail_blend_alpha = 1.0;
    bool static_simplify = true;
    int motion_threshold = 14;
    int motion_erode_px = 1;
    int motion_dilate_px = 2;
    int motion_trail_frames = 3;
    double trail_disable_motion_ratio = 0.30;
    double bg_update_alpha = 0.01;
    double bg_blur_sigma = 1.2;
    int center_clear_size = 100;
  };

  struct AbrConfig
  {
    bool enabled = false;
    double link_bandwidth_kbps = 0.0;
    double latency_target_ms = 300.0;
    int min_bitrate_kbps = 12;
    int max_bitrate_kbps = 0;
    int up_step_kbps = 2;
    int update_interval_frames = 6;
    double hysteresis_ms = 40.0;
  };

  int width = 0;
  int height = 0;
  double fps = 24.0;
  int bitrate_kbps = 40;
  std::size_t max_packet_bytes = kMaxSupportedPacketBytes;
  std::string x264_preset = "auto";
  int keyframe_interval_frames = 24;
  int max_reorder_frames = 8;
  TemporalAggregationConfig temporal_aggregation;
  AbrConfig abr;
};

struct EncodedPacket
{
  std::uint32_t frame_id = 0;
  std::uint16_t fragment_index = 0;
  std::uint16_t fragment_count = 0;
  std::int64_t pts = 0;
  bool keyframe = false;
  std::vector<std::uint8_t> payload;
};

struct DecodedFrame
{
  std::uint32_t frame_id = 0;
  std::int64_t pts = 0;
  bool keyframe = false;
  cv::Mat image_bgr;
};

struct EncoderTelemetry
{
  bool emitted_encoded_frame = false;
  int source_frames_aggregated = 0;
  bool used_temporal_trail = false;
  int requested_bitrate_kbps = 0;
  int next_bitrate_kbps = 0;
  bool bitrate_changed_for_next_frame = false;
  double encode_ms = 0.0;
  double packetize_ms = 0.0;
  double tx_queue_delay_ms = 0.0;
  double tx_serialize_delay_ms = 0.0;
  double tx_total_delay_ms = 0.0;
  double estimated_latency_ms = 0.0;
  double tx_backlog_bytes = 0.0;
  std::size_t encoded_wire_bytes = 0;
};

void validate_config(const CodecConfig & config);

std::size_t packet_wire_size(const EncodedPacket & packet);
std::vector<std::uint8_t> serialize_packet(const EncodedPacket & packet);
EncodedPacket deserialize_packet(const std::vector<std::uint8_t> & wire_bytes);

class PacketEncoder
{
public:
  struct Impl;

  explicit PacketEncoder(CodecConfig config);
  ~PacketEncoder();

  PacketEncoder(PacketEncoder &&) noexcept;
  PacketEncoder & operator=(PacketEncoder &&) noexcept;

  PacketEncoder(const PacketEncoder &) = delete;
  PacketEncoder & operator=(const PacketEncoder &) = delete;

  std::vector<EncodedPacket> encode(
    const cv::Mat & bgr_frame,
    std::optional<std::int64_t> pts = std::nullopt);

  std::vector<EncodedPacket> flush();

  const CodecConfig & config() const noexcept;
  const EncoderTelemetry & last_telemetry() const noexcept;

private:
  std::vector<EncodedPacket> encode_ready_frame(
    const cv::Mat & bgr_frame,
    std::int64_t output_pts,
    int source_frames_aggregated,
    bool used_temporal_trail);
  bool reconfigure_bitrate_kbps(int bitrate_kbps);

  CodecConfig config_;
  std::unique_ptr<Impl> impl_;
  std::uint32_t next_frame_id_ = 0;
  std::int64_t next_pts_ = 0;
  std::int64_t next_source_pts_ = 0;
};

class PacketDecoder
{
public:
  struct Impl;

  explicit PacketDecoder(CodecConfig config);
  ~PacketDecoder();

  PacketDecoder(PacketDecoder &&) noexcept;
  PacketDecoder & operator=(PacketDecoder &&) noexcept;

  PacketDecoder(const PacketDecoder &) = delete;
  PacketDecoder & operator=(const PacketDecoder &) = delete;

  std::vector<DecodedFrame> push_packet(const EncodedPacket & packet);
  std::vector<DecodedFrame> push_serialized_packet(const std::vector<std::uint8_t> & wire_bytes);
  std::vector<DecodedFrame> flush();

  const CodecConfig & config() const noexcept;

private:
  CodecConfig config_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm_image_compression
