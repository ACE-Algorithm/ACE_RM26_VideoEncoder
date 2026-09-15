#ifndef LANDING_POINT_DETECTOR__LANDING_POINT_DETECTOR_NODE_HPP_
#define LANDING_POINT_DETECTOR__LANDING_POINT_DETECTOR_NODE_HPP_

#include <std_msgs/msg/header.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rm_image_compression/codec.hpp"

namespace landing_point_detector
{

class LandingPointDetectorNode : public rclcpp::Node
{
public:
  explicit LandingPointDetectorNode(const rclcpp::NodeOptions & options);

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void dispatchFrame(const cv::Mat & img, const std_msgs::msg::Header & header);
  static int normalizeEvenDimension(int requested, int fallback);
  void initializeCenterCropEncoder(const cv::Mat & frame);
  void publishCenterCrop(const cv::Mat & img, const std_msgs::msg::Header & header);
  void publishNextCenterCropPacket();
  void update_parameter();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  image_transport::Publisher center_crop_img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr center_crop_packet_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;

  bool debug_;
  std::atomic<int> center_crop_size_{300};
  std::atomic<int> center_crop_width_{300};
  std::atomic<int> center_crop_height_{300};
  double center_crop_codec_fps_{12.0};
  double center_crop_packet_send_rate_hz_{12.0};
  int center_crop_bitrate_kbps_{100};
  int center_crop_abr_min_bitrate_kbps_{50};
  int center_crop_abr_max_bitrate_kbps_{100};
  int center_crop_abr_buffer_size_kbits_{30};
  int center_crop_max_packet_bytes_{300};
  int center_crop_keyframe_interval_frames_{24};
  int center_crop_max_reorder_frames_{8};
  std::string center_crop_x264_preset_{"slow"};
  bool center_crop_static_simplify_{true};
  int center_crop_motion_threshold_{14};
  int center_crop_motion_erode_px_{1};
  int center_crop_motion_dilate_px_{2};
  int center_crop_motion_trail_frames_{3};
  double center_crop_trail_disable_motion_ratio_{0.30};
  double center_crop_bg_update_alpha_{0.01};
  double center_crop_bg_blur_sigma_{1.2};
  int center_crop_center_clear_size_{100};
  bool center_crop_center_color_roi_{false};
  bool center_crop_center_bitrate_roi_{false};
  double center_crop_center_bitrate_roi_qoffset_{-0.20};
  std::string center_crop_debug_topic_{"/image/center_crop"};
  std::string center_crop_packet_topic_{
    "/landing_point_detector/center_crop_packets/compressed"};

  std::mutex center_crop_encode_mutex_;
  rm_image_compression::CodecConfig center_crop_codec_config_;
  std::unique_ptr<rm_image_compression::PacketEncoder> center_crop_encoder_;
  std::chrono::duration<double> center_crop_send_period_{
    std::chrono::duration<double>(1.0 / rm_image_compression::kDefaultCodecFps)};
  std::size_t center_crop_accepted_frames_{0};
  std::size_t center_crop_dropped_frames_{0};
  std::size_t center_crop_published_packets_{0};
  std::deque<std::vector<std::uint8_t>> center_crop_queued_packets_;

  bool record_video_ = false;
  rclcpp::TimerBase::SharedPtr center_crop_send_timer_;

};

}  // namespace landing_point_detector

#endif  // LANDING_POINT_DETECTOR__LANDING_POINT_DETECTOR_NODE_HPP_
