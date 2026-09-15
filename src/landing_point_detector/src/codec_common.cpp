#include "rm_image_compression/codec.hpp"

#include "codec_internal.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>

extern "C" {
#include <libavutil/error.h>
}

namespace rm_image_compression
{

namespace
{

std::string to_lower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

void write_u16_be(const std::uint16_t value, std::uint8_t * output)
{
  output[0] = static_cast<std::uint8_t>((value >> 8) & 0xffU);
  output[1] = static_cast<std::uint8_t>(value & 0xffU);
}

std::uint16_t read_u16_be(const std::uint8_t * input)
{
  return static_cast<std::uint16_t>(
    (static_cast<std::uint16_t>(input[0]) << 8) |
    static_cast<std::uint16_t>(input[1]));
}

}  // namespace

void validate_config(const CodecConfig & config)
{
  if (config.width <= 0 || config.height <= 0) {
    throw std::invalid_argument("Codec width/height must be positive");
  }
  if ((config.width % 2) != 0 || (config.height % 2) != 0) {
    throw std::invalid_argument("Codec width/height must be even for yuv420p");
  }
  if (config.fps <= 0.0) {
    throw std::invalid_argument("Codec fps must be positive");
  }
  if (config.bitrate_kbps <= 0) {
    throw std::invalid_argument("Codec bitrate_kbps must be positive");
  }
  if (config.max_packet_bytes <= kPacketHeaderBytes) {
    throw std::invalid_argument("max_packet_bytes must leave room for payload bytes");
  }
  if (config.keyframe_interval_frames <= 0) {
    throw std::invalid_argument("keyframe_interval_frames must be positive");
  }
  if (config.max_reorder_frames <= 0) {
    throw std::invalid_argument("max_reorder_frames must be positive");
  }
  if (
    config.temporal_aggregation.trail_blend_alpha < 0.0 ||
    config.temporal_aggregation.trail_blend_alpha > 1.0)
  {
    throw std::invalid_argument("temporal_aggregation trail_blend_alpha must be in [0, 1]");
  }
  if (config.temporal_aggregation.motion_threshold < 0) {
    throw std::invalid_argument("temporal_aggregation motion_threshold must be non-negative");
  }
  if (config.temporal_aggregation.motion_erode_px < 0) {
    throw std::invalid_argument("temporal_aggregation motion_erode_px must be non-negative");
  }
  if (config.temporal_aggregation.motion_dilate_px < 0) {
    throw std::invalid_argument("temporal_aggregation motion_dilate_px must be non-negative");
  }
  if (config.temporal_aggregation.motion_trail_frames < 0) {
    throw std::invalid_argument("temporal_aggregation motion_trail_frames must be non-negative");
  }
  if (
    config.temporal_aggregation.trail_disable_motion_ratio < 0.0 ||
    config.temporal_aggregation.trail_disable_motion_ratio > 1.0)
  {
    throw std::invalid_argument("temporal_aggregation trail_disable_motion_ratio must be in [0, 1]");
  }
  if (config.temporal_aggregation.bg_update_alpha <= 0.0) {
    throw std::invalid_argument("temporal_aggregation bg_update_alpha must be positive");
  }
  if (config.temporal_aggregation.bg_blur_sigma < 0.0) {
    throw std::invalid_argument("temporal_aggregation bg_blur_sigma must be non-negative");
  }
  if (config.temporal_aggregation.center_clear_size < 0) {
    throw std::invalid_argument("temporal_aggregation center_clear_size must be non-negative");
  }
  if (
    config.temporal_aggregation.center_bitrate_roi_qoffset < -1.0 ||
    config.temporal_aggregation.center_bitrate_roi_qoffset > 1.0)
  {
    throw std::invalid_argument(
            "temporal_aggregation center_bitrate_roi_qoffset must be in [-1, 1]");
  }
  if (config.abr.min_bitrate_kbps <= 0) {
    throw std::invalid_argument("abr min_bitrate_kbps must be positive");
  }
  if (config.abr.max_bitrate_kbps > 0 && config.abr.max_bitrate_kbps < config.abr.min_bitrate_kbps) {
    throw std::invalid_argument("abr max_bitrate_kbps must be >= min_bitrate_kbps");
  }
  if (config.bitrate_kbps < config.abr.min_bitrate_kbps) {
    throw std::invalid_argument("bitrate_kbps must be >= abr min_bitrate_kbps");
  }
  if (config.abr.max_bitrate_kbps > 0 && config.bitrate_kbps > config.abr.max_bitrate_kbps) {
    throw std::invalid_argument("bitrate_kbps must be <= abr max_bitrate_kbps");
  }
  if (config.abr.buffer_size_kbits < 0) {
    throw std::invalid_argument("abr buffer_size_kbits must be non-negative");
  }
}

std::size_t packet_wire_size(const EncodedPacket & packet)
{
  return kPacketHeaderBytes + packet.payload.size();
}

std::vector<std::uint8_t> serialize_packet(const EncodedPacket & packet)
{
  if (packet.payload.empty()) {
    throw std::invalid_argument("packet payload must not be empty");
  }
  std::vector<std::uint8_t> wire_bytes(packet_wire_size(packet), 0U);
  write_u16_be(packet.sequence_id, wire_bytes.data());
  std::copy(packet.payload.begin(), packet.payload.end(), wire_bytes.begin() + static_cast<std::ptrdiff_t>(kPacketHeaderBytes));
  return wire_bytes;
}

EncodedPacket deserialize_packet(const std::vector<std::uint8_t> & wire_bytes)
{
  if (wire_bytes.size() < kPacketHeaderBytes) {
    throw std::invalid_argument("wire packet is smaller than the fixed header");
  }
  EncodedPacket packet;
  packet.sequence_id = read_u16_be(wire_bytes.data());
  packet.payload.assign(wire_bytes.begin() + static_cast<std::ptrdiff_t>(kPacketHeaderBytes), wire_bytes.end());
  if (packet.payload.empty()) {
    throw std::invalid_argument("wire packet has empty payload");
  }

  return packet;
}

namespace detail
{

std::string ffmpeg_error_to_string(const int errnum)
{
  std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
  av_strerror(errnum, buffer.data(), buffer.size());
  return std::string(buffer.data());
}

std::string resolve_preset(std::string requested)
{
  requested = to_lower(std::move(requested));
  if (requested.empty() || requested == "auto") {
    return "slow";
  }

  static const std::array<const char *, 10> valid_presets = {
    "ultrafast", "superfast", "veryfast", "faster", "fast",
    "medium", "slow", "slower", "veryslow", "placebo"
  };
  const bool is_valid = std::any_of(
    valid_presets.begin(), valid_presets.end(),
    [&requested](const char * preset) { return requested == preset; });
  return is_valid ? requested : "slow";
}

std::string build_x264_params()
{
  return "repeat-headers=1:annexb=1:scenecut=0:force-cfr=1";
}

}  // namespace detail

}  // namespace rm_image_compression
