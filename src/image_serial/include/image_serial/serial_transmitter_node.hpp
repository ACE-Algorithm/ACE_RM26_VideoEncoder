#ifndef IMAGE_SERIAL__SERIAL_TRANSMITTER_NODE_HPP_
#define IMAGE_SERIAL__SERIAL_TRANSMITTER_NODE_HPP_

#include <boost/asio.hpp>
#include <deque>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>
#include <vector>

namespace image_serial {

class SerialTransmitterNode : public rclcpp::Node {
public:
    // Component 标准构造函数
    explicit SerialTransmitterNode(const rclcpp::NodeOptions & options);
    
    // 析构函数
    ~SerialTransmitterNode() override;

private:
    // 压缩图像回调函数
    void compressed_topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // 固定频率发送队列中的数据
    void send_timer_callback();
    
    // 串口发送核心函数
    void send_packet(const uint8_t * raw_data, size_t raw_size);

    // 尝试（重新）打开串口，成功后取消重连定时器
    void tryReopen();

    // 启动重连定时器（幂等，已在运行则跳过）
    void startReconnectTimer();

    // Asio 串口对象与 ROS 2 成员变量
    boost::asio::io_context io_;
    boost::asio::serial_port serial_;

    // 串口参数（从 ROS2 参数加载）
    std::string serial_port_;
    int baud_rate_;
    int character_size_;
    std::string stop_bits_str_;
    std::string parity_str_;

    std::string compressed_input_topic_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_subscription_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    std::deque<std::vector<uint8_t>> send_queue_;

    uint8_t seq_ = 0; // 包序号
};

} // namespace image_serial

#endif // IMAGE_SERIAL__SERIAL_TRANSMITTER_NODE_HPP_
