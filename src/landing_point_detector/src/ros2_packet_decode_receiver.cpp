#include "rm_image_compression/codec.hpp"
#include "rm_image_compression/receiver_presentation.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace
{

inline constexpr const char * kPacketFormat = "rm_image_compression/packet_seq_v2";

int normalize_even_dimension(const int requested, const int fallback)
{
  const int raw_value = requested > 0 ? requested : fallback;
  const int clamped_value = std::max(raw_value, 2);
  return (clamped_value % 2 == 0) ? clamped_value : (clamped_value - 1);
}

std::size_t normalize_max_packet_bytes(const int requested)
{
  return static_cast<std::size_t>(std::max(
    requested,
    static_cast<int>(rm_image_compression::kPacketHeaderBytes + 1U)));
}

class Ros2PacketDecodeReceiver : public rclcpp::Node
{
public:
  Ros2PacketDecodeReceiver()
  : Node("ros2_packet_decode_receiver"),
    start_time_(std::chrono::steady_clock::now())
  {
    input_topic_ = this->declare_parameter<std::string>(
      "input_topic", "landing_point_detector/center_crop_packets/compressed"); ///landing_point_detector/center_crop_packets/compressed
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "/rm_image_compression/image_decoded");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "camera");
    width_ = this->declare_parameter<int>("width", 300);
    height_ = this->declare_parameter<int>("height", 300);
    codec_fps_ = this->declare_parameter<double>("codec_fps", 12.0);
    bitrate_kbps_ = this->declare_parameter<int>("bitrate_kbps", 100);
    max_packet_bytes_ = this->declare_parameter<int>("max_packet_bytes", 300);
    keyframe_interval_frames_ = this->declare_parameter<int>("keyframe_interval_frames", 24);
    max_reorder_frames_ = this->declare_parameter<int>("max_reorder_frames", 8);
    display_fps_ = this->declare_parameter<double>("display_fps", 48.0);
    min_display_latency_ms_ = this->declare_parameter<double>("min_display_latency_ms", 400.0);

    codec_config_.width = normalize_even_dimension(width_, 300);
    codec_config_.height = normalize_even_dimension(height_, 300);
    codec_config_.fps = std::max(codec_fps_, 1.0);
    codec_config_.bitrate_kbps = std::max(bitrate_kbps_, 1);
    codec_config_.max_packet_bytes = normalize_max_packet_bytes(max_packet_bytes_);
    codec_config_.keyframe_interval_frames = std::max(keyframe_interval_frames_, 1);
    codec_config_.max_reorder_frames = std::max(max_reorder_frames_, 1);

    decoder_ = std::make_unique<rm_image_compression::PacketDecoder>(codec_config_);

    rm_image_compression::ReceiverPresentationConfig presentation_config;
    presentation_config.display_fps = std::max(display_fps_, 1.0);
    presentation_config.min_presented_latency_ms = std::max(min_display_latency_ms_, 0.0);
    scheduler_ = std::make_unique<rm_image_compression::ReceiverFrameScheduler>(
      presentation_config);

    packet_subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&Ros2PacketDecodeReceiver::handle_packet, this, std::placeholders::_1));
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      output_topic_,
      rclcpp::SensorDataQoS());

    const auto display_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / scheduler_->config().display_fps));
    display_timer_ = this->create_wall_timer(
      display_period,
      std::bind(&Ros2PacketDecodeReceiver::publish_due_frame, this));

    RCLCPP_INFO(
      this->get_logger(),
      "packet receiver ready: %s -> %s, %dx%d, codec_fps=%.2f, display_fps=%.2f, delay=%.1fms",
      input_topic_.c_str(),
      output_topic_.c_str(),
      codec_config_.width,
      codec_config_.height,
      codec_config_.fps,
      scheduler_->config().display_fps,
      scheduler_->config().min_presented_latency_ms);
  }

private:
  double steady_seconds_since_start() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();
  }

  void decode_serialized_packet(
    const std::vector<std::uint8_t> & serialized_packet,
    const double decode_ready_time_sec)
  {
    std::vector<rm_image_compression::DecodedFrame> decoded_frames =
      decoder_->push_serialized_packet(serialized_packet);
    ++received_packet_count_;

    for (auto & frame : decoded_frames) {
      const double source_time_sec =
        rm_image_compression::frame_pts_to_time_sec(frame.pts, codec_config_.fps);
      scheduler_->push_frame(std::move(frame), source_time_sec, decode_ready_time_sec);
      ++decoded_frame_count_;
    }
  }

  void handle_packet(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    if (msg->format != kPacketFormat) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "unexpected packet format: %s (expected %s)",
        msg->format.c_str(),
        kPacketFormat);
      return;
    }

    try {
      const double decode_ready_time_sec = steady_seconds_since_start();
      decode_serialized_packet(msg->data, decode_ready_time_sec);

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "packets=%zu decoded_frames=%zu published_frames=%zu",
        received_packet_count_,
        decoded_frame_count_,
        published_frame_count_);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "failed to decode packet: %s",
        ex.what());
    }
  }

  void publish_due_frame()
  {
    std::optional<rm_image_compression::ReceiverPresentationFrame> presented;
    presented = scheduler_->update(steady_seconds_since_start());

    if (!presented.has_value() || presented->frame.image_bgr.empty()) {
      return;
    }

    cv::Mat output_image = presented->frame.image_bgr;
    if (!output_image.isContinuous()) {
      output_image = output_image.clone();
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = this->now();
    image_msg.header.frame_id = output_frame_id_;
    image_msg.height = static_cast<sensor_msgs::msg::Image::_height_type>(output_image.rows);
    image_msg.width = static_cast<sensor_msgs::msg::Image::_width_type>(output_image.cols);
    image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    image_msg.is_bigendian = false;
    image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(output_image.step[0]);
    const std::size_t byte_count =
      static_cast<std::size_t>(output_image.rows) * output_image.step[0];
    image_msg.data.assign(
      output_image.data,
      output_image.data + static_cast<std::ptrdiff_t>(byte_count));

    image_publisher_->publish(image_msg);
    ++published_frame_count_;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "published frame_id=%u latency=%.1fms",
      presented->frame.frame_id,
      presented->presented_latency_ms);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  int width_ = 300;
  int height_ = 300;
  double codec_fps_ = 12.0;
  int bitrate_kbps_ = 100;
  int max_packet_bytes_ = 300;
  int keyframe_interval_frames_ = 24;
  int max_reorder_frames_ = 8;
  double display_fps_ = 48.0;
  double min_display_latency_ms_ = 400.0;

  std::chrono::steady_clock::time_point start_time_;
  rm_image_compression::CodecConfig codec_config_;
  std::unique_ptr<rm_image_compression::PacketDecoder> decoder_;
  std::unique_ptr<rm_image_compression::ReceiverFrameScheduler> scheduler_;

  std::size_t received_packet_count_ = 0;
  std::size_t decoded_frame_count_ = 0;
  std::size_t published_frame_count_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr packet_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr display_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ros2PacketDecodeReceiver>());
  rclcpp::shutdown();
  return 0;
}
