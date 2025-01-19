#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <map>
#include <string>
#include <vector>
#include <mutex>

using StringConstPtr = std_msgs::msg::String::ConstSharedPtr;

class GuiniverseNode : public rclcpp::Node
{
public:
    GuiniverseNode();

    void SetupWithImageTransport(image_transport::ImageTransport &);

    void run();

private:
    void BarcodeCallback(const StringConstPtr &);
    void TimerCallback();
    void SetMotorStatus(bool status);
    void SetImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding);

    std::vector<image_transport::Subscriber> m_ImageSubscribers;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_BarcodeSubscriber;
    std::map<std::string, size_t> m_Barcodes;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    geometry_msgs::msg::Twist m_TwistMessage;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_GripperPublisher;
    std_msgs::msg::Float32MultiArray m_GripperMessage;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_EnableMotorClient;

    bool m_SetMotorStatusLock = false;
    bool m_ShouldSetMotorStatusTrue = false;
    bool m_ShouldSetMotorStatusFalse = false;
    bool m_IsImageUpdated = false;

    rclcpp::TimerBase::SharedPtr m_Timer;

    int m_Count = 0;
    bool m_IsRunning = true;
};
