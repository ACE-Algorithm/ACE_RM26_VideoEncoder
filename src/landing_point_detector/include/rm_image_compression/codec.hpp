#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace rm_image_compression
{

inline constexpr double kDefaultCodecFps = 12.0;
inline constexpr std::size_t kPacketHeaderBytes = 2;
inline constexpr std::size_t kDefaultPacketBytes = 300;
inline constexpr std::size_t kTransportLengthPrefixBytes = 2;
inline constexpr std::size_t kMaxLengthPrefixedPacketBytes =
  static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max());

struct CodecConfig
{
  struct TemporalAggregationConfig
  {
    bool enabled = false;
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
    bool center_color_roi = false;
    bool center_bitrate_roi = false;
    double center_bitrate_roi_qoffset = -0.20;
  };

  struct AbrConfig
  {
    int min_bitrate_kbps = 50;
    int max_bitrate_kbps = 100;
    int buffer_size_kbits = 30;
  };

  int width = 0;
  int height = 0;
  double fps = kDefaultCodecFps;
  int bitrate_kbps = 100;
  std::size_t max_packet_bytes = kDefaultPacketBytes;
  std::string x264_preset = "slow";
  bool intra_refresh = false;
  int keyframe_interval_frames = 24;
  int max_reorder_frames = 8;
  TemporalAggregationConfig temporal_aggregation;
  AbrConfig abr;
};

struct EncodedPacket
{
  std::uint16_t sequence_id = 0;
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
    std::int64_t pts);

  std::vector<EncodedPacket> flush();

  const CodecConfig & config() const noexcept;

private:
  std::vector<EncodedPacket> encode_ready_frame(
    const cv::Mat & bgr_frame,
    std::int64_t output_pts);

  CodecConfig config_;
  std::unique_ptr<Impl> impl_;
  std::uint32_t next_frame_id_ = 0;
  std::int64_t next_pts_ = 0;
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

class TransportPacketPacker
{
public:
  explicit TransportPacketPacker(
    std::size_t transport_packet_bytes = kDefaultPacketBytes,
    std::size_t max_inner_packet_bytes = kDefaultPacketBytes - kTransportLengthPrefixBytes);

  void push_serialized_packet(std::vector<std::uint8_t> serialized_packet);
  bool has_pending_packets() const noexcept;
  std::size_t pending_packet_count() const noexcept;
  std::size_t pending_serialized_bytes() const noexcept;

  // Returns an empty vector when no packet is pending.
  // Non-empty output is always exactly transport_packet_bytes long.
  std::vector<std::uint8_t> pop_transport_packet();

private:
  std::size_t transport_packet_bytes_ = kDefaultPacketBytes;
  std::size_t max_inner_packet_bytes_ = kDefaultPacketBytes - kTransportLengthPrefixBytes;
  std::vector<std::vector<std::uint8_t>> pending_packets_;
  std::size_t pending_serialized_bytes_ = 0;
};

std::vector<std::vector<std::uint8_t>> unpack_transport_packet(
  const std::vector<std::uint8_t> & transport_packet,
  std::size_t max_inner_packet_bytes = kDefaultPacketBytes);

}  // namespace rm_image_compression
