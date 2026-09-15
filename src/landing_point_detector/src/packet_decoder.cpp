#include "rm_image_compression/codec.hpp"

#include "codec_internal.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace rm_image_compression
{

struct PacketDecoder::Impl
{
  struct PendingFrame
  {
    std::uint32_t frame_id = 0;
    std::int64_t pts = 0;
    bool keyframe = false;
  };

  AVCodecContext * codec_context = nullptr;
  AVCodecParserContext * parser = nullptr;
  SwsContext * sws_context = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * packet = nullptr;
  std::uint8_t * converted_bgr_data[4] = {nullptr, nullptr, nullptr, nullptr};
  int converted_bgr_linesize[4] = {0, 0, 0, 0};
  int converted_bgr_width = 0;
  int converted_bgr_height = 0;
  std::deque<PendingFrame> pending_frames;
  int source_width = 0;
  int source_height = 0;
  AVPixelFormat source_format = AV_PIX_FMT_NONE;
  std::uint32_t next_frame_id = 0;
  std::int64_t next_pts = 0;
  bool has_expected_sequence = false;
  std::uint16_t expected_sequence_id = 0;
  bool waiting_for_keyframe = true;
};

namespace
{

bool sequence_is_newer(const std::uint16_t lhs, const std::uint16_t rhs)
{
  const std::uint16_t delta = static_cast<std::uint16_t>(lhs - rhs);
  return delta != 0U && delta < 0x8000U;
}

std::size_t find_start_code_prefix(
  const std::uint8_t * data,
  const std::size_t size,
  const std::size_t offset,
  std::size_t * prefix_len)
{
  if (prefix_len == nullptr) {
    throw std::invalid_argument("find_start_code_prefix() received null prefix_len");
  }
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("find_start_code_prefix() received null data with non-zero size");
  }

  for (std::size_t i = offset; (i + 3U) <= size; ++i) {
    if (
      (i + 4U) <= size &&
      data[i] == 0U &&
      data[i + 1U] == 0U &&
      data[i + 2U] == 0U &&
      data[i + 3U] == 1U)
    {
      *prefix_len = 4U;
      return i;
    }
    if (data[i] == 0U && data[i + 1U] == 0U && data[i + 2U] == 1U) {
      *prefix_len = 3U;
      return i;
    }
  }

  return size;
}

bool access_unit_has_idr(const std::uint8_t * data, const std::size_t size)
{
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("access_unit_has_idr() received null data with non-zero size");
  }

  std::size_t search_offset = 0U;
  while (search_offset < size) {
    std::size_t prefix_len = 0U;
    const std::size_t prefix_offset = find_start_code_prefix(
      data,
      size,
      search_offset,
      &prefix_len);
    if (prefix_offset >= size) {
      break;
    }

    const std::size_t nal_header_offset = prefix_offset + prefix_len;
    if (nal_header_offset >= size) {
      break;
    }

    const std::uint8_t nal_type = data[nal_header_offset] & 0x1fU;
    if (nal_type == 5U) {
      return true;
    }

    search_offset = nal_header_offset + 1U;
  }

  return false;
}

void free_decoder_impl(PacketDecoder::Impl & impl)
{
  if (impl.converted_bgr_data[0] != nullptr) {
    av_freep(&impl.converted_bgr_data[0]);
  }
  if (impl.parser != nullptr) {
    av_parser_close(impl.parser);
    impl.parser = nullptr;
  }
  if (impl.packet != nullptr) {
    av_packet_free(&impl.packet);
  }
  if (impl.frame != nullptr) {
    av_frame_free(&impl.frame);
  }
  if (impl.codec_context != nullptr) {
    avcodec_free_context(&impl.codec_context);
  }
  if (impl.sws_context != nullptr) {
    sws_freeContext(impl.sws_context);
    impl.sws_context = nullptr;
  }
}

void reset_parser(PacketDecoder::Impl * impl)
{
  if (impl == nullptr) {
    throw std::invalid_argument("reset_parser() received null impl");
  }
  if (impl->parser != nullptr) {
    av_parser_close(impl->parser);
    impl->parser = nullptr;
  }
  impl->parser = av_parser_init(AV_CODEC_ID_H264);
  if (impl->parser == nullptr) {
    throw std::runtime_error("Failed to allocate H.264 parser");
  }
}

void reset_decoder_after_loss(PacketDecoder::Impl * impl)
{
  if (impl == nullptr) {
    throw std::invalid_argument("reset_decoder_after_loss() received null impl");
  }
  if (impl->codec_context != nullptr) {
    avcodec_flush_buffers(impl->codec_context);
  }
  impl->pending_frames.clear();
  impl->waiting_for_keyframe = true;
  reset_parser(impl);
}

void ensure_converted_bgr_buffer(
  PacketDecoder::Impl & impl,
  const int width,
  const int height)
{
  if (
    impl.converted_bgr_data[0] != nullptr &&
    impl.converted_bgr_width == width &&
    impl.converted_bgr_height == height)
  {
    return;
  }

  if (impl.converted_bgr_data[0] != nullptr) {
    av_freep(&impl.converted_bgr_data[0]);
    std::fill(
      std::begin(impl.converted_bgr_data),
      std::end(impl.converted_bgr_data),
      nullptr);
    std::fill(
      std::begin(impl.converted_bgr_linesize),
      std::end(impl.converted_bgr_linesize),
      0);
  }

  const int ret = av_image_alloc(
    impl.converted_bgr_data,
    impl.converted_bgr_linesize,
    width,
    height,
    AV_PIX_FMT_BGR24,
    32);
  if (ret < 0) {
    throw std::runtime_error(
      "Failed to allocate aligned decoder BGR buffer: " +
      detail::ffmpeg_error_to_string(ret));
  }

  impl.converted_bgr_width = width;
  impl.converted_bgr_height = height;
}

void ensure_sws_context(PacketDecoder::Impl & impl, const AVFrame * frame)
{
  const AVPixelFormat frame_format = static_cast<AVPixelFormat>(frame->format);
  if (
    impl.sws_context != nullptr &&
    impl.source_width == frame->width &&
    impl.source_height == frame->height &&
    impl.source_format == frame_format)
  {
    return;
  }

  if (impl.sws_context != nullptr) {
    sws_freeContext(impl.sws_context);
    impl.sws_context = nullptr;
  }

  impl.sws_context = sws_getContext(
    frame->width, frame->height, frame_format,
    frame->width, frame->height, AV_PIX_FMT_BGR24,
    SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (impl.sws_context == nullptr) {
    throw std::runtime_error("Failed to create decoder swscale context");
  }

  impl.source_width = frame->width;
  impl.source_height = frame->height;
  impl.source_format = frame_format;
}

std::vector<DecodedFrame> drain_decoded_frames(PacketDecoder::Impl & impl)
{
  std::vector<DecodedFrame> frames;
  while (true) {
    const int ret_receive = avcodec_receive_frame(impl.codec_context, impl.frame);
    if (ret_receive == AVERROR(EAGAIN) || ret_receive == AVERROR_EOF) {
      break;
    }
    if (ret_receive < 0) {
      throw std::runtime_error(
        "Failed to receive decoded frame: " + detail::ffmpeg_error_to_string(ret_receive));
    }

    ensure_sws_context(impl, impl.frame);
    ensure_converted_bgr_buffer(impl, impl.frame->width, impl.frame->height);

    sws_scale(
      impl.sws_context,
      impl.frame->data,
      impl.frame->linesize,
      0,
      impl.frame->height,
      impl.converted_bgr_data,
      impl.converted_bgr_linesize);

    const cv::Mat wrapped_bgr(
      impl.frame->height,
      impl.frame->width,
      CV_8UC3,
      impl.converted_bgr_data[0],
      static_cast<std::size_t>(impl.converted_bgr_linesize[0]));

    DecodedFrame output;
    if (!impl.pending_frames.empty()) {
      const PacketDecoder::Impl::PendingFrame meta = impl.pending_frames.front();
      impl.pending_frames.pop_front();
      output.frame_id = meta.frame_id;
      output.pts = meta.pts;
      output.keyframe = meta.keyframe;
    } else {
      output.frame_id = impl.next_frame_id++;
      output.pts = (impl.frame->pts == AV_NOPTS_VALUE) ? impl.next_pts++ : impl.frame->pts;
      output.keyframe = (impl.frame->flags & AV_PKT_FLAG_KEY) != 0;
    }
    output.image_bgr = wrapped_bgr.clone();
    frames.push_back(std::move(output));
  }
  return frames;
}

std::vector<DecodedFrame> decode_access_unit(
  PacketDecoder::Impl * impl,
  const std::uint8_t * access_unit,
  const std::size_t access_unit_size)
{
  if (impl == nullptr) {
    throw std::invalid_argument("decode_access_unit() received null impl");
  }
  if (access_unit == nullptr && access_unit_size != 0U) {
    throw std::invalid_argument("decode_access_unit() received null data with non-zero size");
  }
  if (access_unit_size == 0U) {
    return {};
  }

  const bool keyframe = access_unit_has_idr(access_unit, access_unit_size);
  if (impl->waiting_for_keyframe && !keyframe) {
    return {};
  }

  const int ret_packet = av_new_packet(impl->packet, static_cast<int>(access_unit_size));
  if (ret_packet < 0) {
    throw std::runtime_error(
      "Failed to allocate decoder packet: " + detail::ffmpeg_error_to_string(ret_packet));
  }

  std::memcpy(impl->packet->data, access_unit, access_unit_size);
  impl->packet->pts = impl->next_pts;
  impl->packet->dts = impl->next_pts;
  impl->packet->flags = keyframe ? AV_PKT_FLAG_KEY : 0;

  const int ret_send = avcodec_send_packet(impl->codec_context, impl->packet);
  av_packet_unref(impl->packet);
  if (ret_send < 0) {
    throw std::runtime_error(
      "Failed to send packet to decoder: " + detail::ffmpeg_error_to_string(ret_send));
  }

  impl->pending_frames.push_back({
    impl->next_frame_id++,
    impl->next_pts++,
    keyframe,
  });
  impl->waiting_for_keyframe = false;
  return drain_decoded_frames(*impl);
}

void append_frames(
  std::vector<DecodedFrame> * output,
  std::vector<DecodedFrame> decoded)
{
  if (output == nullptr) {
    throw std::invalid_argument("append_frames() received null output");
  }
  output->insert(
    output->end(),
    std::make_move_iterator(decoded.begin()),
    std::make_move_iterator(decoded.end()));
}

std::vector<DecodedFrame> feed_parser(
  PacketDecoder::Impl * impl,
  const std::uint8_t * payload,
  std::size_t payload_size)
{
  if (impl == nullptr) {
    throw std::invalid_argument("feed_parser() received null impl");
  }
  if (payload == nullptr && payload_size != 0U) {
    throw std::invalid_argument("feed_parser() received null payload with non-zero size");
  }

  std::vector<DecodedFrame> output;
  const std::uint8_t * cursor = payload;
  while (payload_size > 0U) {
    std::uint8_t * parsed_data = nullptr;
    int parsed_size = 0;
    const int consumed = av_parser_parse2(
      impl->parser,
      impl->codec_context,
      &parsed_data,
      &parsed_size,
      cursor,
      static_cast<int>(payload_size),
      AV_NOPTS_VALUE,
      AV_NOPTS_VALUE,
      0);
    if (consumed < 0) {
      throw std::runtime_error(
        "Failed to parse H.264 byte stream: " + detail::ffmpeg_error_to_string(consumed));
    }

    cursor += consumed;
    payload_size -= static_cast<std::size_t>(consumed);

    if (parsed_size > 0) {
      append_frames(
        &output,
        decode_access_unit(
          impl,
          parsed_data,
          static_cast<std::size_t>(parsed_size)));
    }

    if (consumed == 0 && parsed_size == 0) {
      break;
    }
  }

  return output;
}

std::vector<DecodedFrame> flush_parser(PacketDecoder::Impl * impl)
{
  if (impl == nullptr) {
    throw std::invalid_argument("flush_parser() received null impl");
  }

  std::vector<DecodedFrame> output;
  while (true) {
    std::uint8_t * parsed_data = nullptr;
    int parsed_size = 0;
    const int consumed = av_parser_parse2(
      impl->parser,
      impl->codec_context,
      &parsed_data,
      &parsed_size,
      nullptr,
      0,
      AV_NOPTS_VALUE,
      AV_NOPTS_VALUE,
      0);
    if (consumed < 0) {
      throw std::runtime_error(
        "Failed while flushing H.264 parser: " + detail::ffmpeg_error_to_string(consumed));
    }
    (void)consumed;
    if (parsed_size <= 0) {
      break;
    }
    append_frames(
      &output,
      decode_access_unit(
        impl,
        parsed_data,
        static_cast<std::size_t>(parsed_size)));
  }

  return output;
}

}  // namespace

PacketDecoder::PacketDecoder(CodecConfig config)
: config_(std::move(config)),
  impl_(std::make_unique<Impl>())
{
  validate_config(config_);

  const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (codec == nullptr) {
    throw std::runtime_error("Unable to find an H.264 decoder");
  }

  impl_->codec_context = avcodec_alloc_context3(codec);
  if (impl_->codec_context == nullptr) {
    throw std::runtime_error("Failed to allocate decoder context");
  }

  const int ret_open = avcodec_open2(impl_->codec_context, codec, nullptr);
  if (ret_open < 0) {
    const std::string message = detail::ffmpeg_error_to_string(ret_open);
    free_decoder_impl(*impl_);
    throw std::runtime_error("Failed to open decoder: " + message);
  }

  impl_->frame = av_frame_alloc();
  impl_->packet = av_packet_alloc();
  if (impl_->frame == nullptr || impl_->packet == nullptr) {
    free_decoder_impl(*impl_);
    throw std::runtime_error("Failed to allocate decoder frame or packet");
  }

  try {
    reset_parser(impl_.get());
  } catch (...) {
    free_decoder_impl(*impl_);
    throw;
  }
}

PacketDecoder::~PacketDecoder()
{
  if (impl_ != nullptr) {
    free_decoder_impl(*impl_);
  }
}

PacketDecoder::PacketDecoder(PacketDecoder &&) noexcept = default;
PacketDecoder & PacketDecoder::operator=(PacketDecoder &&) noexcept = default;

std::vector<DecodedFrame> PacketDecoder::push_packet(const EncodedPacket & packet)
{
  if (packet.payload.empty()) {
    throw std::invalid_argument("push_packet() received an empty payload");
  }
  if (packet_wire_size(packet) > config_.max_packet_bytes) {
    throw std::invalid_argument("push_packet() received a packet larger than the configured limit");
  }

  if (!impl_->has_expected_sequence) {
    impl_->expected_sequence_id = static_cast<std::uint16_t>(packet.sequence_id + 1U);
    impl_->has_expected_sequence = true;
  } else if (packet.sequence_id == impl_->expected_sequence_id) {
    impl_->expected_sequence_id = static_cast<std::uint16_t>(impl_->expected_sequence_id + 1U);
  } else if (sequence_is_newer(packet.sequence_id, impl_->expected_sequence_id)) {
    reset_decoder_after_loss(impl_.get());
    impl_->expected_sequence_id = static_cast<std::uint16_t>(packet.sequence_id + 1U);
    impl_->has_expected_sequence = true;
  } else {
    return {};
  }

  return feed_parser(
    impl_.get(),
    packet.payload.data(),
    packet.payload.size());
}

std::vector<DecodedFrame> PacketDecoder::push_serialized_packet(const std::vector<std::uint8_t> & wire_bytes)
{
  return push_packet(deserialize_packet(wire_bytes));
}

std::vector<DecodedFrame> PacketDecoder::flush()
{
  std::vector<DecodedFrame> output = flush_parser(impl_.get());

  const int ret_send = avcodec_send_packet(impl_->codec_context, nullptr);
  if (ret_send < 0 && ret_send != AVERROR_EOF) {
    throw std::runtime_error(
      "Failed to flush decoder: " + detail::ffmpeg_error_to_string(ret_send));
  }

  append_frames(&output, drain_decoded_frames(*impl_));
  return output;
}

const CodecConfig & PacketDecoder::config() const noexcept
{
  return config_;
}

}  // namespace rm_image_compression
