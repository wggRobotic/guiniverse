#pragma once

#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

struct DataCaptureSystemSection final
{
    std::string SectionName;
    std::map<std::string, size_t> Data;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Subscriber;
};

class DataCaptureSystem final
{
public:
    DataCaptureSystem(std::shared_ptr<rclcpp::Node> Node);

    void AddSection(const std::string& Name, const std::string& TopicName);

    void Callback(size_t index, const std_msgs::msg::String::SharedPtr msg);

    void ImGuiPanels();

private:
    std::shared_ptr<rclcpp::Node> m_Node;

    std::mutex m_Mutex;
    std::vector<DataCaptureSystemSection> m_Sections;
};
