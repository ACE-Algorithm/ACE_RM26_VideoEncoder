#include "rm_image_compression/codec.hpp"
#include "rm_image_compression/temporal_frame_composer.hpp"

#include "codec_internal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/opt.h>
#include <libavutil/rational.h>
#include <libswscale/swscale.h>
}

namespace rm_image_compression
{

struct PacketEncoder::Impl
{
  struct PendingFrame
  {
    std::uint32_t frame_id = 0;
    std::int64_t pts = 0;
  };

  AVCodecContext * codec_context = nullptr;
  SwsContext * sws_context = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * packet = nullptr;
  std::deque<PendingFrame> pending_frames;
  std::vector<std::uint8_t> pending_stream_bytes;
  std::size_t pending_stream_offset = 0U;
  std::uint16_t next_sequence_id = 0U;
  bool force_next_keyframe = true;
  std::unique_ptr<TemporalFrameComposer> temporal_frame_composer;
};

namespace
{

void free_encoder_impl(PacketEncoder::Impl & impl)
{
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

void normalize_codec_config(CodecConfig * config)
{
  if (config == nullptr) {
    throw std::invalid_argument("normalize_codec_config() received a null config");
  }
  CodecConfig::AbrConfig & abr_config = config->abr;
  CodecConfig::TemporalAggregationConfig & temporal = config->temporal_aggregation;
  temporal.trail_blend_alpha = std::clamp(temporal.trail_blend_alpha, 0.0, 1.0);
  temporal.motion_threshold = std::max(temporal.motion_threshold, 0);
  temporal.motion_erode_px = std::clamp(temporal.motion_erode_px, 0, 20);
  temporal.motion_dilate_px = std::clamp(temporal.motion_dilate_px, 0, 20);
  temporal.motion_trail_frames = std::clamp(temporal.motion_trail_frames, 0, 15);
  temporal.trail_disable_motion_ratio =
    std::clamp(temporal.trail_disable_motion_ratio, 0.0, 1.0);
  temporal.bg_update_alpha = std::clamp(temporal.bg_update_alpha, 0.001, 0.2);
  temporal.bg_blur_sigma = std::max(temporal.bg_blur_sigma, 0.0);
  temporal.center_clear_size = std::max(temporal.center_clear_size, 0);
  temporal.center_bitrate_roi_qoffset =
    std::clamp(temporal.center_bitrate_roi_qoffset, -1.0, 1.0);
  abr_config.min_bitrate_kbps = std::max(abr_config.min_bitrate_kbps, 1);
  if (abr_config.max_bitrate_kbps <= 0) {
    abr_config.max_bitrate_kbps = config->bitrate_kbps;
  }
  abr_config.max_bitrate_kbps = std::max(abr_config.max_bitrate_kbps, abr_config.min_bitrate_kbps);
  abr_config.buffer_size_kbits = std::max(abr_config.buffer_size_kbits, 0);
}

std::unique_ptr<PacketEncoder::Impl> create_encoder_impl(const CodecConfig & config)
{
  auto impl = std::make_unique<PacketEncoder::Impl>();

  const AVCodec * codec = avcodec_find_encoder_by_name("libx264");
  if (codec == nullptr) {
    codec = avcodec_find_encoder(AV_CODEC_ID_H264);
  }
  if (codec == nullptr) {
    throw std::runtime_error("Unable to find an H.264 encoder");
  }

  impl->codec_context = avcodec_alloc_context3(codec);
  if (impl->codec_context == nullptr) {
    throw std::runtime_error("Failed to allocate encoder context");
  }

  const AVRational fps_q = av_d2q(config.fps, 100000);
  const int keyframe_interval = std::max(1, config.keyframe_interval_frames);
  impl->codec_context->codec_id = codec->id;
  impl->codec_context->codec_type = AVMEDIA_TYPE_VIDEO;
  impl->codec_context->width = config.width;
  impl->codec_context->height = config.height;
  impl->codec_context->pix_fmt = AV_PIX_FMT_YUV420P;
  impl->codec_context->time_base = av_inv_q(fps_q);
  impl->codec_context->framerate = fps_q;
  impl->codec_context->bit_rate = static_cast<std::int64_t>(config.bitrate_kbps) * 1000LL;
  const std::int64_t min_rate =
    static_cast<std::int64_t>(config.abr.min_bitrate_kbps) * 1000LL;
  const std::int64_t max_rate =
    static_cast<std::int64_t>(config.abr.max_bitrate_kbps) * 1000LL;
  int buffer_size_kbits = config.abr.buffer_size_kbits;
  if (buffer_size_kbits <= 0) {
    buffer_size_kbits = std::max(
      1,
      static_cast<int>(std::lround(static_cast<double>(config.bitrate_kbps) * 0.3)));
  }
  impl->codec_context->rc_min_rate = min_rate;
  impl->codec_context->rc_max_rate = max_rate;
  impl->codec_context->rc_buffer_size = static_cast<std::int64_t>(buffer_size_kbits) * 1000LL;
  impl->codec_context->gop_size = keyframe_interval;
  impl->codec_context->max_b_frames = 0;
  impl->codec_context->thread_count = 0;
  impl->codec_context->keyint_min = 1;

  const std::string preset = detail::resolve_preset(config.x264_preset);
  av_opt_set(impl->codec_context->priv_data, "preset", preset.c_str(), 0);
  av_opt_set(impl->codec_context->priv_data, "tune", "zerolatency", 0);
  av_opt_set(impl->codec_context->priv_data, "forced-idr", "1", 0);
  av_opt_set(
    impl->codec_context->priv_data,
    "intra-refresh",
    config.intra_refresh ? "1" : "0",
    0);
  const std::string x264_params = detail::build_x264_params();
  av_opt_set(impl->codec_context->priv_data, "x264-params", x264_params.c_str(), 0);

  const int ret_open = avcodec_open2(impl->codec_context, codec, nullptr);
  if (ret_open < 0) {
    const std::string message = detail::ffmpeg_error_to_string(ret_open);
    free_encoder_impl(*impl);
    throw std::runtime_error("Failed to open encoder: " + message);
  }

  impl->frame = av_frame_alloc();
  impl->packet = av_packet_alloc();
  if (impl->frame == nullptr || impl->packet == nullptr) {
    free_encoder_impl(*impl);
    throw std::runtime_error("Failed to allocate encoder frame or packet");
  }

  impl->frame->format = impl->codec_context->pix_fmt;
  impl->frame->width = impl->codec_context->width;
  impl->frame->height = impl->codec_context->height;

  const int ret_buffer = av_frame_get_buffer(impl->frame, 32);
  if (ret_buffer < 0) {
    const std::string message = detail::ffmpeg_error_to_string(ret_buffer);
    free_encoder_impl(*impl);
    throw std::runtime_error("Failed to allocate encoder frame buffer: " + message);
  }

  impl->sws_context = sws_getContext(
    config.width, config.height, AV_PIX_FMT_BGR24,
    config.width, config.height, AV_PIX_FMT_YUV420P,
    SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (impl->sws_context == nullptr) {
    free_encoder_impl(*impl);
    throw std::runtime_error("Failed to create encoder swscale context");
  }

  impl->force_next_keyframe = true;
  if (config.temporal_aggregation.enabled) {
    impl->temporal_frame_composer = std::make_unique<TemporalFrameComposer>(
      config.fps,
      config.temporal_aggregation);
  }
  return impl;
}

std::size_t pending_stream_size(const PacketEncoder::Impl & impl)
{
  if (impl.pending_stream_offset >= impl.pending_stream_bytes.size()) {
    return 0U;
  }
  return impl.pending_stream_bytes.size() - impl.pending_stream_offset;
}

void compact_pending_stream(PacketEncoder::Impl * impl)
{
  if (impl == nullptr) {
    throw std::invalid_argument("compact_pending_stream() received null impl");
  }
  if (impl->pending_stream_offset == 0U) {
    return;
  }
  if (impl->pending_stream_offset >= impl->pending_stream_bytes.size()) {
    impl->pending_stream_bytes.clear();
    impl->pending_stream_offset = 0U;
    return;
  }
  if (
    impl->pending_stream_offset >= 4096U &&
    (impl->pending_stream_offset * 2U) >= impl->pending_stream_bytes.size())
  {
    impl->pending_stream_bytes.erase(
      impl->pending_stream_bytes.begin(),
      impl->pending_stream_bytes.begin() +
      static_cast<std::ptrdiff_t>(impl->pending_stream_offset));
    impl->pending_stream_offset = 0U;
  }
}

void append_access_unit_bytes(
  PacketEncoder::Impl * impl,
  const std::uint8_t * data,
  const std::size_t size)
{
  if (impl == nullptr) {
    throw std::invalid_argument("append_access_unit_bytes() received null impl");
  }
  if (data == nullptr && size != 0U) {
    throw std::invalid_argument("append_access_unit_bytes() received null data with non-zero size");
  }
  if (size == 0U) {
    return;
  }
  compact_pending_stream(impl);
  impl->pending_stream_bytes.insert(
    impl->pending_stream_bytes.end(),
    data,
    data + static_cast<std::ptrdiff_t>(size));
}

std::vector<EncodedPacket> drain_transport_packets(
  PacketEncoder::Impl * impl,
  const CodecConfig & config,
  const bool allow_partial_tail,
  const std::uint32_t frame_id = 0U,
  const std::int64_t pts = 0,
  const bool keyframe = false,
  const bool with_frame_meta = false)
{
  if (impl == nullptr) {
    throw std::invalid_argument("drain_transport_packets() received null impl");
  }
  const std::size_t max_payload_bytes = config.max_packet_bytes - kPacketHeaderBytes;
  if (max_payload_bytes == 0U) {
    throw std::runtime_error("max_payload_bytes must be positive");
  }

  std::vector<EncodedPacket> packets;
  while (true) {
    const std::size_t available = pending_stream_size(*impl);
    if (available < max_payload_bytes && !(allow_partial_tail && available > 0U)) {
      break;
    }
    const std::size_t chunk_size = std::min(max_payload_bytes, available);
    EncodedPacket packet;
    packet.sequence_id = impl->next_sequence_id++;
    if (with_frame_meta) {
      packet.frame_id = frame_id;
      packet.pts = pts;
      packet.keyframe = keyframe;
      packet.fragment_index = 0;
      packet.fragment_count = 1;
    }
    packet.payload.assign(
      impl->pending_stream_bytes.begin() + static_cast<std::ptrdiff_t>(impl->pending_stream_offset),
      impl->pending_stream_bytes.begin() + static_cast<std::ptrdiff_t>(impl->pending_stream_offset + chunk_size));
    impl->pending_stream_offset += chunk_size;
    packets.push_back(std::move(packet));
    compact_pending_stream(impl);
  }

  return packets;
}

void apply_center_bitrate_roi_side_data(const CodecConfig & config, AVFrame * frame)
{
  if (frame == nullptr) {
    throw std::invalid_argument("apply_center_bitrate_roi_side_data() received null frame");
  }

  av_frame_remove_side_data(frame, AV_FRAME_DATA_REGIONS_OF_INTEREST);
  if (!config.temporal_aggregation.center_bitrate_roi) {
    return;
  }

  const int roi_width = std::max(1, frame->width / 3);
  const int roi_height = std::max(1, frame->height / 3);
  const int left = std::max(0, (frame->width - roi_width) / 2);
  const int top = std::max(0, (frame->height - roi_height) / 2);
  const int right = std::min(frame->width, left + roi_width);
  const int bottom = std::min(frame->height, top + roi_height);

  AVFrameSideData * roi_side_data = av_frame_new_side_data(
    frame,
    AV_FRAME_DATA_REGIONS_OF_INTEREST,
    sizeof(AVRegionOfInterest));
  if (roi_side_data == nullptr || roi_side_data->data == nullptr) {
    throw std::runtime_error("Failed to allocate AV_FRAME_DATA_REGIONS_OF_INTEREST side data");
  }

  auto * roi = reinterpret_cast<AVRegionOfInterest *>(roi_side_data->data);
  std::memset(roi, 0, sizeof(AVRegionOfInterest));
  roi->self_size = sizeof(AVRegionOfInterest);
  roi->top = top;
  roi->bottom = bottom;
  roi->left = left;
  roi->right = right;
  roi->qoffset = av_d2q(config.temporal_aggregation.center_bitrate_roi_qoffset, 1000000);
  if (roi->qoffset.den == 0) {
    roi->qoffset = AVRational{0, 1};
  }
}

}  // namespace

PacketEncoder::PacketEncoder(CodecConfig config)
: config_(std::move(config)),
  impl_(nullptr)
{
  normalize_codec_config(&config_);
  validate_config(config_);
  impl_ = create_encoder_impl(config_);
}

PacketEncoder::~PacketEncoder()
{
  if (impl_ != nullptr) {
    free_encoder_impl(*impl_);
  }
}

PacketEncoder::PacketEncoder(PacketEncoder &&) noexcept = default;
PacketEncoder & PacketEncoder::operator=(PacketEncoder &&) noexcept = default;

std::vector<EncodedPacket> PacketEncoder::encode(
  const cv::Mat & bgr_frame,
  const std::int64_t source_timestamp_ns)
{
  if (bgr_frame.empty()) {
    throw std::invalid_argument("encode() received an empty frame");
  }
  if (bgr_frame.type() != CV_8UC3) {
    throw std::invalid_argument("encode() expects CV_8UC3 input");
  }
  if (bgr_frame.cols != config_.width || bgr_frame.rows != config_.height) {
    throw std::invalid_argument("encode() received a frame with unexpected dimensions");
  }

  //做背景差分
  if (impl_->temporal_frame_composer != nullptr) {
    const std::optional<TemporalCompositionResult> composed =
      impl_->temporal_frame_composer->push_frame(bgr_frame, source_timestamp_ns);
    if (!composed.has_value()) {
      return {};
    }

    const std::int64_t output_pts = next_pts_++;
    return encode_ready_frame(
      composed->frame_bgr,
      output_pts);
  }

  const std::int64_t frame_pts = next_pts_++;
  return encode_ready_frame(bgr_frame, frame_pts);
}

std::vector<EncodedPacket> PacketEncoder::encode_ready_frame(
  const cv::Mat & bgr_frame,
  const std::int64_t frame_pts)
{
  if (bgr_frame.empty()) {
    throw std::invalid_argument("encode_ready_frame() received an empty frame");
  }
  if (bgr_frame.type() != CV_8UC3) {
    throw std::invalid_argument("encode_ready_frame() expects CV_8UC3 input");
  }
  if (bgr_frame.cols != config_.width || bgr_frame.rows != config_.height) {
    throw std::invalid_argument("encode_ready_frame() received a frame with unexpected dimensions");
  }

  //防止和编码器仍在使用的旧帧冲突
  const int ret_writable = av_frame_make_writable(impl_->frame);
  if (ret_writable < 0) {
    throw std::runtime_error(
      "Encoder frame is not writable: " + detail::ffmpeg_error_to_string(ret_writable));
  }

  const std::uint32_t frame_id = next_frame_id_++;
  const std::uint8_t * src_slices[] = {bgr_frame.data};
  const int src_stride[] = {static_cast<int>(bgr_frame.step[0])};
  //转换格式，将图像从BGR转换为YUV420P格式
  sws_scale(
    impl_->sws_context,
    src_slices,
    src_stride,
    0,
    config_.height,
    impl_->frame->data,
    impl_->frame->linesize);
  apply_center_bitrate_roi_side_data(config_, impl_->frame);

  impl_->frame->pict_type = impl_->force_next_keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
  impl_->frame->pts = frame_pts;
  const int ret_send = avcodec_send_frame(impl_->codec_context, impl_->frame);
  if (ret_send < 0) {
    throw std::runtime_error(
      "Failed to send frame to encoder: " + detail::ffmpeg_error_to_string(ret_send));
  }
  impl_->force_next_keyframe = false;
  impl_->pending_frames.push_back({frame_id, frame_pts});
  std::vector<EncodedPacket> output_packets;
  while (true) {
    const int ret_receive = avcodec_receive_packet(impl_->codec_context, impl_->packet);
    if (ret_receive == AVERROR(EAGAIN) || ret_receive == AVERROR_EOF) {
      break;
    }
    if (ret_receive < 0) {
      throw std::runtime_error(
        "Failed to receive encoded packet: " + detail::ffmpeg_error_to_string(ret_receive));
    }
    if (impl_->pending_frames.empty()) {
      av_packet_unref(impl_->packet);
      throw std::runtime_error("Encoder produced a packet without pending frame metadata");
    }

    const Impl::PendingFrame meta = impl_->pending_frames.front();
    impl_->pending_frames.pop_front();
    const bool keyframe = (impl_->packet->flags & AV_PKT_FLAG_KEY) != 0;
    append_access_unit_bytes(
      impl_.get(),
      impl_->packet->data,
      static_cast<std::size_t>(impl_->packet->size));
    std::vector<EncodedPacket> frame_packets = drain_transport_packets(
      impl_.get(),
      config_,
      false,
      meta.frame_id,
      meta.pts,
      keyframe,
      true);
    output_packets.insert(
      output_packets.end(),
      std::make_move_iterator(frame_packets.begin()),
      std::make_move_iterator(frame_packets.end()));
    av_packet_unref(impl_->packet);
  }
  return output_packets;
}

std::vector<EncodedPacket> PacketEncoder::flush()
{
  std::vector<EncodedPacket> output_packets;

  if (impl_->temporal_frame_composer != nullptr) {
    const std::optional<TemporalCompositionResult> composed =
      impl_->temporal_frame_composer->flush();
    if (composed.has_value()) {
      std::vector<EncodedPacket> pending_packets = encode_ready_frame(
        composed->frame_bgr,
        next_pts_++);
      output_packets.insert(
        output_packets.end(),
        std::make_move_iterator(pending_packets.begin()),
        std::make_move_iterator(pending_packets.end()));
    }
  }

  const int ret_send = avcodec_send_frame(impl_->codec_context, nullptr);
  if (ret_send < 0 && ret_send != AVERROR_EOF) {
    throw std::runtime_error(
      "Failed to flush encoder: " + detail::ffmpeg_error_to_string(ret_send));
  }

  while (true) {
    const int ret_receive = avcodec_receive_packet(impl_->codec_context, impl_->packet);
    if (ret_receive == AVERROR(EAGAIN) || ret_receive == AVERROR_EOF) {
      break;
    }
    if (ret_receive < 0) {
      throw std::runtime_error(
        "Failed to receive flushed packet: " + detail::ffmpeg_error_to_string(ret_receive));
    }
    if (impl_->pending_frames.empty()) {
      av_packet_unref(impl_->packet);
      throw std::runtime_error("Encoder flush produced a packet without pending frame metadata");
    }

    const Impl::PendingFrame meta = impl_->pending_frames.front();
    impl_->pending_frames.pop_front();
    const bool keyframe = (impl_->packet->flags & AV_PKT_FLAG_KEY) != 0;
    append_access_unit_bytes(
      impl_.get(),
      impl_->packet->data,
      static_cast<std::size_t>(impl_->packet->size));
    std::vector<EncodedPacket> frame_packets = drain_transport_packets(
      impl_.get(),
      config_,
      false,
      meta.frame_id,
      meta.pts,
      keyframe,
      true);
    output_packets.insert(
      output_packets.end(),
      std::make_move_iterator(frame_packets.begin()),
      std::make_move_iterator(frame_packets.end()));
    av_packet_unref(impl_->packet);
  }

  std::vector<EncodedPacket> tail_packets = drain_transport_packets(
    impl_.get(),
    config_,
    true);
  output_packets.insert(
    output_packets.end(),
    std::make_move_iterator(tail_packets.begin()),
    std::make_move_iterator(tail_packets.end()));

  return output_packets;
}

const CodecConfig & PacketEncoder::config() const noexcept
{
  return config_;
}

}  // namespace rm_image_compression
