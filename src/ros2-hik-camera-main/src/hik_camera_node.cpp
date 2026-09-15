#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <hik_sdk/hikrobot/shared_hik_camera_params.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    camera_index_ = this->declare_parameter("camera_index", 0);
    RCLCPP_INFO(this->get_logger(), "Using camera_index = %d", camera_index_);

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[camera_index_]);

    MV_CC_OpenDevice(camera_handle_);

    // bool bSetBoolValue = 1;
    // nRet = MV_CC_SetBoolValue(camera_handle_,"ReverseX",bSetBoolValue);
    // nRet = MV_CC_SetBoolValue(camera_handle_,"ReverseY",bSetBoolValue);


    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);
    MV_CC_SetBayerCvtQuality(camera_handle_, 2);
    MV_CC_SetEnumValue(camera_handle_, "PixelFormat", 0x01080009);
    MV_CC_SetEnumValue(camera_handle_, "ADCBitDepth", 12);
    MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", 240);
    MV_CC_SetEnumValue(camera_handle_, "Trigger Mode", MV_TRIGGER_MODE_OFF);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    this->declare_parameter<std::vector<std::string>>(
      "camera_info_urls", std::vector<std::string>{});
    if (io::shared_hik_camera_params::loadCameraInfoMessageForIndex(
        this, camera_info_manager_.get(), camera_index_, camera_info_msg_))
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Loaded camera calibration for camera_index=%d from %s",
        camera_index_,
        camera_info_url.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to load camera calibration for camera_index=%d from %s",
        camera_index_,
        camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = camera_info_msg_.header.frame_id.empty()
        ? io::shared_hik_camera_params::getLogicalCameraOpticalFrame(camera_index_)
        : camera_info_msg_.header.frame_id;
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == nRet) {
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          // Flip vertically if enabled
          if (flip_vertically_) {
            flipImageVertically(image_msg_.data.data(), out_frame.stFrameInfo.nWidth, out_frame.stFrameInfo.nHeight);
          }

          image_msg_.header.stamp = this->now();
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_conut_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // Binning — 必须在 Width/Height 之前设置，否则传感器尺寸范围校验会失败
    param_desc.description = "Binning mode (1=off, 2=2x2, 4=4x4). Must be set before exposure/gain";
    param_desc.integer_range[0].from_value = 1;
    param_desc.integer_range[0].to_value = 4;
    binning_ = this->declare_parameter("binning", 1, param_desc);
    {
      int ret = MV_CC_SetEnumValue(camera_handle_, "BinningHorizontal", static_cast<unsigned int>(binning_));
      RCLCPP_INFO(this->get_logger(), "Set BinningHorizontal=%d, ret=0x%x", binning_, ret);
      ret = MV_CC_SetEnumValue(camera_handle_, "BinningVertical", static_cast<unsigned int>(binning_));
      RCLCPP_INFO(this->get_logger(), "Set BinningVertical=%d, ret=0x%x", binning_, ret);
      // 设置 Width/Height=0 让相机自动使用 binning 后的最大分辨率
      MV_CC_SetIntValue(camera_handle_, "Width", 0);
      MV_CC_SetIntValue(camera_handle_, "Height", 0);
    }
    RCLCPP_INFO(this->get_logger(), "Binning: %d", binning_);

    // Flip vertically
    param_desc.description = "Flip image vertically (upside down)";
    flip_vertically_ = this->declare_parameter("flip_vertically", false, param_desc);
    RCLCPP_INFO(this->get_logger(), "Flip vertically: %s", flip_vertically_ ? "true" : "false");
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "binning") {
        int new_binning = param.as_int();
        if (new_binning >= 1) {
          binning_ = new_binning;
          MV_CC_SetEnumValue(camera_handle_, "BinningHorizontal", static_cast<unsigned int>(binning_));
          MV_CC_SetEnumValue(camera_handle_, "BinningVertical", static_cast<unsigned int>(binning_));
          MV_CC_SetIntValue(camera_handle_, "Width", 0);
          MV_CC_SetIntValue(camera_handle_, "Height", 0);
          RCLCPP_INFO(this->get_logger(), "Binning set to: %d", binning_);
        } else {
          result.successful = false;
          result.reason = "Binning must be >= 1";
        }
      } else if (param.get_name() == "flip_vertically") {
        flip_vertically_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Flip vertically set to: %s", flip_vertically_ ? "true" : "false");
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void * camera_handle_;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  int camera_index_{0};

  int fail_conut_ = 0;
  std::thread capture_thread_;
  int binning_ = 1;
  bool flip_vertically_ = false;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  void flipImageVertically(uint8_t * image_data, int width, int height)
  {
    int row_size = width * 3;  // 3 bytes per pixel (RGB8)
    uint8_t * temp_row = new uint8_t[row_size];

    // Flip vertically (upside down)
    for (int i = 0; i < height / 2; ++i) {
      uint8_t * top_row = image_data + i * row_size;
      uint8_t * bottom_row = image_data + (height - 1 - i) * row_size;

      std::memcpy(temp_row, top_row, row_size);
      std::memcpy(top_row, bottom_row, row_size);
      std::memcpy(bottom_row, temp_row, row_size);
    }

    // Flip horizontally (mirror left-right) for each row
    uint8_t * temp_pixel = new uint8_t[3];
    for (int i = 0; i < height; ++i) {
      uint8_t * row = image_data + i * row_size;
      for (int j = 0; j < width / 2; ++j) {
        uint8_t * left_pixel = row + j * 3;
        uint8_t * right_pixel = row + (width - 1 - j) * 3;

        std::memcpy(temp_pixel, left_pixel, 3);
        std::memcpy(left_pixel, right_pixel, 3);
        std::memcpy(right_pixel, temp_pixel, 3);
      }
    }

    delete[] temp_row;
    delete[] temp_pixel;
  }
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
