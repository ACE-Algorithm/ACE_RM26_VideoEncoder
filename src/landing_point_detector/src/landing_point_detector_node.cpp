#include "../include/landing_point_detector/landing_point_detector_node.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace
{
inline constexpr const char * kPacketFormat = "rm_image_compression/packet_seq_v2";

}  // namespace

namespace landing_point_detector
{

LandingPointDetectorNode::LandingPointDetectorNode(const rclcpp::NodeOptions & options)
: Node("landing_point_detector", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting LandingPointDetectorNode!");

  this->declare_parameter<bool>("debug", true);
  this->declare_parameter<bool>("record_video",false);
  const auto image_topic = this->declare_parameter<std::string>("image_topic", "/image_raw");
  center_crop_size_.store(this->declare_parameter<int>("center_crop_size", 300));
  center_crop_width_.store(this->declare_parameter<int>("center_crop_width", 300));
  center_crop_height_.store(this->declare_parameter<int>("center_crop_height", 300));
  this->declare_parameter<bool>("enable_decode_node", false);
  center_crop_packet_topic_ = this->declare_parameter<std::string>(
    "center_crop_packet_topic", "/landing_point_detector/center_crop_packets/compressed");
  center_crop_packet_send_rate_hz_ = this->declare_parameter<double>(
    "center_crop_packet_send_rate_hz", 48);
  center_crop_codec_fps_ = this->declare_parameter<double>(
    "center_crop_codec_fps", 12);
  center_crop_bitrate_kbps_ = this->declare_parameter<int>("center_crop_bitrate_kbps", 100);
  center_crop_abr_min_bitrate_kbps_ =
    this->declare_parameter<int>("center_crop_abr_min_bitrate_kbps", 50);
  center_crop_abr_max_bitrate_kbps_ =
    this->declare_parameter<int>("center_crop_abr_max_bitrate_kbps", 100);
  center_crop_abr_buffer_size_kbits_ =
    this->declare_parameter<int>("center_crop_abr_buffer_size_kbits", 30);
  center_crop_max_packet_bytes_ =
    this->declare_parameter<int>("center_crop_max_packet_bytes", 300);
  center_crop_keyframe_interval_frames_ =
    this->declare_parameter<int>("center_crop_keyframe_interval_frames", 24);
  center_crop_max_reorder_frames_ =
    this->declare_parameter<int>("center_crop_max_reorder_frames", 8);
  center_crop_x264_preset_ =
    this->declare_parameter<std::string>("center_crop_x264_preset", "slow");
  center_crop_static_simplify_ =
    this->declare_parameter<bool>("center_crop_static_simplify", true);
  center_crop_motion_threshold_ =
    this->declare_parameter<int>("center_crop_motion_threshold", 14);
  center_crop_motion_erode_px_ =
    this->declare_parameter<int>("center_crop_motion_erode_px", 1);
  center_crop_motion_dilate_px_ =
    this->declare_parameter<int>("center_crop_motion_dilate_px", 2);
  center_crop_motion_trail_frames_ =
    this->declare_parameter<int>("center_crop_motion_trail_frames", 3);
  center_crop_trail_disable_motion_ratio_ =
    this->declare_parameter<double>("center_crop_trail_disable_motion_ratio", 0.30);
  center_crop_bg_update_alpha_ =
    this->declare_parameter<double>("center_crop_bg_update_alpha", 0.01);
  center_crop_bg_blur_sigma_ =
    this->declare_parameter<double>("center_crop_bg_blur_sigma", 1.2);
  center_crop_center_clear_size_ =
    this->declare_parameter<int>("center_crop_center_clear_size", 100);
  center_crop_center_color_roi_ =
    this->declare_parameter<bool>("center_crop_center_color_roi", false);
  center_crop_center_bitrate_roi_ =
    this->declare_parameter<bool>("center_crop_center_bitrate_roi", false);
  center_crop_center_bitrate_roi_qoffset_ =
    this->declare_parameter<double>("center_crop_center_bitrate_roi_qoffset", -0.20);
  center_crop_debug_topic_ = this->declare_parameter<std::string>(
    "center_crop_debug_topic", "/image/center_crop");

  debug_ = this->get_parameter("debug").as_bool();
  center_crop_img_pub_ = image_transport::create_publisher(this, center_crop_debug_topic_);
  raw_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/landing_point_detector/image_raw", rclcpp::SensorDataQoS());
  center_crop_packet_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    center_crop_packet_topic_, rclcpp::SensorDataQoS());
  const double clamped_send_fps = std::max(center_crop_packet_send_rate_hz_, 1.0);
  center_crop_send_period_ = std::chrono::duration<double>(1.0 / clamped_send_fps);
  center_crop_send_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(center_crop_send_period_),
    std::bind(&LandingPointDetectorNode::publishNextCenterCropPacket, this));
  RCLCPP_INFO(
    this->get_logger(),
    "Center crop compressed topic: %s, size=%d, roi=%dx%d, packet_send_rate_hz=%.2f, codec_fps=%.2f, bitrate=%d kbps",
    center_crop_packet_topic_.c_str(),
    center_crop_size_.load(),
    center_crop_width_.load(),
    center_crop_height_.load(),
    clamped_send_fps,
    std::max(center_crop_codec_fps_, 1.0),
    center_crop_bitrate_kbps_);
  image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/image_raw/compressed/landing_point", 10);
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic, rclcpp::SensorDataQoS(),
    std::bind(&LandingPointDetectorNode::imageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Image source: ROS2 topic (%s)", image_topic.c_str());
}

void LandingPointDetectorNode::update_parameter()
{
  const int new_center_crop_size = this->get_parameter("center_crop_size").as_int();
  const int new_center_crop_width = this->get_parameter("center_crop_width").as_int();
  const int new_center_crop_height = this->get_parameter("center_crop_height").as_int();
  const double new_center_crop_packet_send_rate_hz =
    this->get_parameter("center_crop_packet_send_rate_hz").as_double();
  const double new_center_crop_codec_fps =
    this->get_parameter("center_crop_codec_fps").as_double();
  const int new_center_crop_bitrate_kbps =
    this->get_parameter("center_crop_bitrate_kbps").as_int();
  const int new_center_crop_abr_min_bitrate_kbps =
    this->get_parameter("center_crop_abr_min_bitrate_kbps").as_int();
  const int new_center_crop_abr_max_bitrate_kbps =
    this->get_parameter("center_crop_abr_max_bitrate_kbps").as_int();
  const int new_center_crop_abr_buffer_size_kbits =
    this->get_parameter("center_crop_abr_buffer_size_kbits").as_int();
  const int new_center_crop_max_packet_bytes =
    this->get_parameter("center_crop_max_packet_bytes").as_int();
  const int new_center_crop_keyframe_interval_frames =
    this->get_parameter("center_crop_keyframe_interval_frames").as_int();
  const int new_center_crop_max_reorder_frames =
    this->get_parameter("center_crop_max_reorder_frames").as_int();
  const std::string new_center_crop_x264_preset =
    this->get_parameter("center_crop_x264_preset").as_string();
  const bool new_center_crop_static_simplify =
    this->get_parameter("center_crop_static_simplify").as_bool();
  const int new_center_crop_motion_threshold =
    this->get_parameter("center_crop_motion_threshold").as_int();
  const int new_center_crop_motion_erode_px =
    this->get_parameter("center_crop_motion_erode_px").as_int();
  const int new_center_crop_motion_dilate_px =
    this->get_parameter("center_crop_motion_dilate_px").as_int();
  const int new_center_crop_motion_trail_frames =
    this->get_parameter("center_crop_motion_trail_frames").as_int();
  const double new_center_crop_trail_disable_motion_ratio =
    this->get_parameter("center_crop_trail_disable_motion_ratio").as_double();
  const double new_center_crop_bg_update_alpha =
    this->get_parameter("center_crop_bg_update_alpha").as_double();
  const double new_center_crop_bg_blur_sigma =
    this->get_parameter("center_crop_bg_blur_sigma").as_double();
  const int new_center_crop_center_clear_size =
    this->get_parameter("center_crop_center_clear_size").as_int();
  const bool new_center_crop_center_color_roi =
    this->get_parameter("center_crop_center_color_roi").as_bool();
  const bool new_center_crop_center_bitrate_roi =
    this->get_parameter("center_crop_center_bitrate_roi").as_bool();
  const double new_center_crop_center_bitrate_roi_qoffset =
    this->get_parameter("center_crop_center_bitrate_roi_qoffset").as_double();
  const bool center_crop_packet_send_rate_hz_changed =
    new_center_crop_packet_send_rate_hz != center_crop_packet_send_rate_hz_;
  const bool center_crop_encoder_config_changed =
    new_center_crop_width != center_crop_width_.load() ||
    new_center_crop_height != center_crop_height_.load() ||
    new_center_crop_codec_fps != center_crop_codec_fps_ ||
    new_center_crop_bitrate_kbps != center_crop_bitrate_kbps_ ||
    new_center_crop_abr_min_bitrate_kbps != center_crop_abr_min_bitrate_kbps_ ||
    new_center_crop_abr_max_bitrate_kbps != center_crop_abr_max_bitrate_kbps_ ||
    new_center_crop_abr_buffer_size_kbits != center_crop_abr_buffer_size_kbits_ ||
    new_center_crop_max_packet_bytes != center_crop_max_packet_bytes_ ||
    new_center_crop_keyframe_interval_frames != center_crop_keyframe_interval_frames_ ||
    new_center_crop_max_reorder_frames != center_crop_max_reorder_frames_ ||
    new_center_crop_x264_preset != center_crop_x264_preset_ ||
    new_center_crop_static_simplify != center_crop_static_simplify_ ||
    new_center_crop_motion_threshold != center_crop_motion_threshold_ ||
    new_center_crop_motion_erode_px != center_crop_motion_erode_px_ ||
    new_center_crop_motion_dilate_px != center_crop_motion_dilate_px_ ||
    new_center_crop_motion_trail_frames != center_crop_motion_trail_frames_ ||
    new_center_crop_trail_disable_motion_ratio != center_crop_trail_disable_motion_ratio_ ||
    new_center_crop_bg_update_alpha != center_crop_bg_update_alpha_ ||
    new_center_crop_bg_blur_sigma != center_crop_bg_blur_sigma_ ||
    new_center_crop_center_clear_size != center_crop_center_clear_size_ ||
    new_center_crop_center_color_roi != center_crop_center_color_roi_ ||
    new_center_crop_center_bitrate_roi != center_crop_center_bitrate_roi_ ||
    new_center_crop_center_bitrate_roi_qoffset != center_crop_center_bitrate_roi_qoffset_;

  center_crop_size_.store(new_center_crop_size);
  center_crop_width_.store(new_center_crop_width);
  center_crop_height_.store(new_center_crop_height);
  center_crop_packet_send_rate_hz_ = new_center_crop_packet_send_rate_hz;
  center_crop_codec_fps_ = new_center_crop_codec_fps;
  center_crop_bitrate_kbps_ = new_center_crop_bitrate_kbps;
  center_crop_abr_min_bitrate_kbps_ = new_center_crop_abr_min_bitrate_kbps;
  center_crop_abr_max_bitrate_kbps_ = new_center_crop_abr_max_bitrate_kbps;
  center_crop_abr_buffer_size_kbits_ = new_center_crop_abr_buffer_size_kbits;
  center_crop_max_packet_bytes_ = new_center_crop_max_packet_bytes;
  center_crop_keyframe_interval_frames_ = new_center_crop_keyframe_interval_frames;
  center_crop_max_reorder_frames_ = new_center_crop_max_reorder_frames;
  center_crop_x264_preset_ = new_center_crop_x264_preset;
  center_crop_static_simplify_ = new_center_crop_static_simplify;
  center_crop_motion_threshold_ = new_center_crop_motion_threshold;
  center_crop_motion_erode_px_ = new_center_crop_motion_erode_px;
  center_crop_motion_dilate_px_ = new_center_crop_motion_dilate_px;
  center_crop_motion_trail_frames_ = new_center_crop_motion_trail_frames;
  center_crop_trail_disable_motion_ratio_ = new_center_crop_trail_disable_motion_ratio;
  center_crop_bg_update_alpha_ = new_center_crop_bg_update_alpha;
  center_crop_bg_blur_sigma_ = new_center_crop_bg_blur_sigma;
  center_crop_center_clear_size_ = new_center_crop_center_clear_size;
  center_crop_center_color_roi_ = new_center_crop_center_color_roi;
  center_crop_center_bitrate_roi_ = new_center_crop_center_bitrate_roi;
  center_crop_center_bitrate_roi_qoffset_ = new_center_crop_center_bitrate_roi_qoffset;

  debug_ = this->get_parameter("debug").as_bool();
  record_video_ = this->get_parameter("record_video").as_bool();
  //参数有变，重置定时器，编码器
  if (center_crop_packet_send_rate_hz_changed) {
    const double clamped_send_fps = std::max(center_crop_packet_send_rate_hz_, 1.0);
    center_crop_send_period_ = std::chrono::duration<double>(1.0 / clamped_send_fps);
    center_crop_send_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(center_crop_send_period_),
      std::bind(&LandingPointDetectorNode::publishNextCenterCropPacket, this));
  }
  if (center_crop_encoder_config_changed) {
      std::lock_guard<std::mutex> lock(center_crop_encode_mutex_);
      center_crop_encoder_.reset();
      center_crop_queued_packets_.clear();
      RCLCPP_INFO(this->get_logger(), "Center crop encoder parameters changed; reinitializing");
  }
}

void LandingPointDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto img = cv_bridge::toCvShare(msg, "bgr8")->image;
  dispatchFrame(img, msg->header);
}

void LandingPointDetectorNode::dispatchFrame(const cv::Mat & img, const std_msgs::msg::Header & header)
{
  update_parameter();

  //发布相机原图
  if(record_video_ || debug_)
  {
      // 创建压缩图像消息
      sensor_msgs::msg::CompressedImage compressed_msg;
      compressed_msg.header = header;
      compressed_msg.header.stamp = this->get_clock()->now(); // 用当前时间戳，避免时钟不同步导致的显示问题
      compressed_msg.header.frame_id = header.frame_id;
      compressed_msg.format = "jpeg";  

      // 将cv::Mat编码为JPEG格式
      std::vector<uchar> buf;
      cv::imencode(".jpg", img, buf);
      compressed_msg.data = buf;

      // 发布压缩图像
      image_pub_->publish(compressed_msg);
      auto raw_img_msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
      raw_image_pub_->publish(*raw_img_msg);
  }
  publishCenterCrop(img, header);
}

int LandingPointDetectorNode::normalizeEvenDimension(const int requested, const int fallback)
{
  const int raw_value = requested > 0 ? requested : fallback;
  const int clamped_value = std::max(raw_value, 2);
  return (clamped_value % 2 == 0) ? clamped_value : (clamped_value - 1);
}

void LandingPointDetectorNode::initializeCenterCropEncoder(const cv::Mat & frame)
{
  const int requested_width = center_crop_width_.load() > 0 ?
    center_crop_width_.load() : center_crop_size_.load();
  const int requested_height = center_crop_height_.load() > 0 ?
    center_crop_height_.load() : center_crop_size_.load();
  center_crop_codec_config_.width = normalizeEvenDimension(requested_width, frame.cols);
  center_crop_codec_config_.height = normalizeEvenDimension(requested_height, frame.rows);
  center_crop_codec_config_.fps = std::max(center_crop_codec_fps_, 1.0);
  center_crop_codec_config_.bitrate_kbps = std::max(center_crop_bitrate_kbps_, 1);
  center_crop_codec_config_.abr.min_bitrate_kbps = std::max(center_crop_abr_min_bitrate_kbps_, 1);
  center_crop_codec_config_.abr.max_bitrate_kbps = std::max(center_crop_abr_max_bitrate_kbps_, 1);
  center_crop_codec_config_.abr.buffer_size_kbits = std::max(center_crop_abr_buffer_size_kbits_, 0);
  center_crop_codec_config_.x264_preset = center_crop_x264_preset_;
  center_crop_codec_config_.keyframe_interval_frames =
    std::max(center_crop_keyframe_interval_frames_, 1);
  center_crop_codec_config_.max_reorder_frames =
    std::max(center_crop_max_reorder_frames_, 1);
  center_crop_codec_config_.max_packet_bytes = static_cast<std::size_t>(std::max(center_crop_max_packet_bytes_,
      static_cast<int>(rm_image_compression::kPacketHeaderBytes + 1U)));

  center_crop_codec_config_.temporal_aggregation.enabled = true;
  center_crop_codec_config_.temporal_aggregation.static_simplify = center_crop_static_simplify_;
  center_crop_codec_config_.temporal_aggregation.motion_threshold = center_crop_motion_threshold_;
  center_crop_codec_config_.temporal_aggregation.motion_erode_px = center_crop_motion_erode_px_;
  center_crop_codec_config_.temporal_aggregation.motion_dilate_px = center_crop_motion_dilate_px_;
  center_crop_codec_config_.temporal_aggregation.motion_trail_frames = center_crop_motion_trail_frames_;
  center_crop_codec_config_.temporal_aggregation.trail_disable_motion_ratio =
    center_crop_trail_disable_motion_ratio_;
  center_crop_codec_config_.temporal_aggregation.bg_update_alpha = center_crop_bg_update_alpha_;
  center_crop_codec_config_.temporal_aggregation.bg_blur_sigma = center_crop_bg_blur_sigma_;
  center_crop_codec_config_.temporal_aggregation.center_clear_size = center_crop_center_clear_size_;
  center_crop_codec_config_.temporal_aggregation.center_color_roi = center_crop_center_color_roi_;
  center_crop_codec_config_.temporal_aggregation.center_bitrate_roi = center_crop_center_bitrate_roi_;
  center_crop_codec_config_.temporal_aggregation.center_bitrate_roi_qoffset =
    center_crop_center_bitrate_roi_qoffset_;

  center_crop_encoder_ =
    std::make_unique<rm_image_compression::PacketEncoder>(center_crop_codec_config_);

  RCLCPP_INFO(
    this->get_logger(),
    "Center crop encoder initialized: %dx%d @ %.2f fps, bitrate=%d kbps, abr[min=%d max=%d buf=%d], max_packet_bytes=%zu, temporal=%s, color_roi=%s, bitrate_roi=%s, roi_qoffset=%.2f",
    center_crop_codec_config_.width,
    center_crop_codec_config_.height,
    center_crop_codec_config_.fps,
    center_crop_codec_config_.bitrate_kbps,
    center_crop_codec_config_.abr.min_bitrate_kbps,
    center_crop_codec_config_.abr.max_bitrate_kbps,
    center_crop_codec_config_.abr.buffer_size_kbits,
    center_crop_codec_config_.max_packet_bytes,
    center_crop_codec_config_.temporal_aggregation.enabled ? "on" : "off",
    center_crop_codec_config_.temporal_aggregation.center_color_roi ? "on" : "off",
    center_crop_codec_config_.temporal_aggregation.center_bitrate_roi ? "on" : "off",
    center_crop_codec_config_.temporal_aggregation.center_bitrate_roi_qoffset);
}

// 从输入帧裁出中心区域 -> 缩放 -> (可选)发布调试图 -> 送入 H.264 编码器并把产出的包加入发送队列。
void LandingPointDetectorNode::publishCenterCrop(const cv::Mat & img, const std_msgs::msg::Header & header)
{
  if (img.empty()) {
    return;
  }
  update_parameter();
  const int requested_crop_size = center_crop_size_.load();
  if (requested_crop_size <= 0) {
    return;
  }

  const int crop_w = std::min(requested_crop_size, img.cols);
  const int crop_h = std::min(requested_crop_size, img.rows);
  const int crop_x = std::max(0, (img.cols - crop_w) / 2);
  const int crop_y = std::max(0, (img.rows - crop_h) / 2);
  const cv::Rect center_roi(crop_x, crop_y, crop_w, crop_h);
  cv::Mat center_crop = img(center_roi).clone();
  const int requested_width = center_crop_width_.load() > 0 ?
    center_crop_width_.load() : requested_crop_size;
  const int requested_height = center_crop_height_.load() > 0 ?
    center_crop_height_.load() : requested_crop_size;
  //将1000*1000像素画面调整为300*300，像素量变低，相机视野不变
  if (
    requested_width > 0 && requested_height > 0 &&
    (center_crop.cols != requested_width || center_crop.rows != requested_height))
  {
    cv::resize(
      center_crop,
      center_crop,
      cv::Size(requested_width, requested_height),
      0.0,
      0.0,
      cv::INTER_LINEAR);
  }
  // debug 模式下把裁剪后的原始（未压缩）图像发到 center_crop_debug_topic，便于本地预览效果
  if (debug_) {
    std_msgs::msg::Header center_crop_header = header;
    center_crop_header.stamp = this->get_clock()->now();
    auto center_crop_msg = cv_bridge::CvImage(center_crop_header, "bgr8", center_crop).toImageMsg();
    center_crop_img_pub_.publish(center_crop_msg);
  }

  // 时间戳为 0 说明帧没有有效时间信息，编码器的 PTS 依赖它，无法编码，直接丢弃
  if (header.stamp.sec == 0 && header.stamp.nanosec == 0U) {
    ++center_crop_dropped_frames_;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "center crop frame timestamp is zero; dropping frame");
    return;
  }
  // 把 ROS 时间戳换算成纳秒级整数，作为编码器的显示时间戳(PTS)
  const std::int64_t pts =
    static_cast<std::int64_t>(header.stamp.sec) * 1000000000LL +
    static_cast<std::int64_t>(header.stamp.nanosec);

  // 编码器/发送队列同时被发送定时器线程访问，需要加锁
  std::lock_guard<std::mutex> lock(center_crop_encode_mutex_);

  try {
    // 编码器初始化
    if (!center_crop_encoder_) {
      initializeCenterCropEncoder(center_crop);
    }

    cv::Mat resized_frame;
    const cv::Mat * frame_to_encode = &center_crop;
    //确保图像清晰度是300*300
    if (
      center_crop.cols != center_crop_codec_config_.width ||
      center_crop.rows != center_crop_codec_config_.height)
    {
      cv::resize(
        center_crop,
        resized_frame,
        cv::Size(center_crop_codec_config_.width, center_crop_codec_config_.height),
        0.0,
        0.0,
        cv::INTER_LINEAR);
      frame_to_encode = &resized_frame;
    }

    // 编码当前帧，产出的分包先放入队列，由发送定时器按节奏取出发布
    const std::vector<rm_image_compression::EncodedPacket> packets =
      center_crop_encoder_->encode(*frame_to_encode, pts);
    for (const auto & packet : packets) {
      center_crop_queued_packets_.push_back(
        rm_image_compression::serialize_packet(packet));
    }

    ++center_crop_accepted_frames_;
    const std::size_t queued_packets = center_crop_queued_packets_.size();

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "center_crop accepted=%zu dropped=%zu published_packets=%zu queued_packets=%zu",
      center_crop_accepted_frames_,
      center_crop_dropped_frames_,
      center_crop_published_packets_,
      queued_packets);
  } catch (const std::exception & ex) {
    // 编码异常后编码器内部状态可能已损坏，重置编码器和队列，下一帧会重新初始化
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "failed to encode center crop image: %s",
      ex.what());
    center_crop_encoder_.reset();
    center_crop_queued_packets_.clear();
  }
}

void LandingPointDetectorNode::publishNextCenterCropPacket()
{
  sensor_msgs::msg::CompressedImage packet_msg;
  {
    std::lock_guard<std::mutex> lock(center_crop_encode_mutex_);
    if (center_crop_queued_packets_.empty()) {
      return;
    }
    packet_msg.data = std::move(center_crop_queued_packets_.front());
    center_crop_queued_packets_.pop_front();
  }

  packet_msg.header.stamp = this->now();
  packet_msg.format = kPacketFormat;
  center_crop_packet_pub_->publish(packet_msg);
  ++center_crop_published_packets_;
}

}  // namespace landing_point_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(landing_point_detector::LandingPointDetectorNode)
