#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <chrono>
#include <mutex>
#include <map>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <guiniverse/shared_data.hpp>
#include <guiniverse/node.hpp>

#define MAKE_CALLBACK(INDEX) [this, INDEX](const sensor_msgs::msg::Image::ConstSharedPtr &msg) { SetImage(INDEX, msg->data, msg->width, msg->height, msg->step, msg->encoding); }

using namespace std::chrono_literals;

GuiniverseNode::GuiniverseNode()
    : Node("guiniverse"), m_Count(0), m_IsRunning(true)
{
    m_Timer = this->create_wall_timer(100ms, std::bind(&GuiniverseNode::TimerCallback, this));
    m_TwistPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_GripperPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("gripper", 10);

    m_BarcodeSubscriber = this->create_subscription<std_msgs::msg::String>(
        "barcode", 10, std::bind(&GuiniverseNode::BarcodeCallback, this, std::placeholders::_1));

    m_EnableMotorClient = this->create_client<std_srvs::srv::SetBool>("/eduard/enable");

    m_TurtleTwistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
}

void GuiniverseNode::SetImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding)
{
    std::lock_guard<std::mutex> data_lock(image_mutex);
    if (index >= shared_image_data.size())
    {
        std::cerr << "Index out of range: " << index << std::endl;
        return;
    }

    auto &[image, dirty] = shared_image_data[index];
    image = {width, height, step, encoding, data};
    dirty = true;
}

void GuiniverseNode::SetupWithImageTransport(image_transport::ImageTransport &it)
{
    m_ImageSubscribers.resize(IMAGE_TOPICS.size());

    {
        std::lock_guard<std::mutex> lock_image_data(image_mutex);
        shared_image_data.resize(IMAGE_TOPICS.size());
    }

    for (size_t i = 0; i < IMAGE_TOPICS.size(); i++)
        m_ImageSubscribers[i] = it.subscribe(std::string(IMAGE_TOPICS.at(i)), 10, MAKE_CALLBACK(i));
}

void GuiniverseNode::run()
{
    rclcpp::Rate rate(30);
    while (rclcpp::ok() && m_IsRunning)
    {
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex);
            shared_data = "ROS2 is running: " + std::to_string(this->now().seconds());
        }

        if (!running)
        {
            m_IsRunning = running;
        }
        rclcpp::spin_some(shared_from_this());

        if (!m_EnableMotorClientWaiting)
        {
            bool is_set;
            bool status;

            {
                std::lock_guard<std::mutex> lock(motor_enable_service_mutex);
                is_set = motor_enable_service_is_set_status;
                status = motor_enable_service_set_status;                
            }
            if (is_set)
            {
                if (!m_EnableMotorClient->service_is_ready()) 
                    RCLCPP_WARN(this->get_logger(), "Enable service is not available.");

                else {
                
                    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
                    request->data = status;

                    if (status)
                        RCLCPP_INFO(this->get_logger(), "Enabeling ...");
                    else
                        RCLCPP_INFO(this->get_logger(), "Disabling ...");

                    m_EnableMotorClient->async_send_request(request, std::bind(&GuiniverseNode::EnableMotorClientCallback, this, std::placeholders::_1));

                    m_EnableMotorClientWaiting = true;
                    m_EnableMotorClientTimeSent = this->now().seconds();
                }
            }

        }
        else if (m_EnableMotorClientWaiting && m_EnableMotorClientTimeSent + 5 < this->now().seconds()) {
            RCLCPP_INFO(this->get_logger(), "Enable Service didn't respond in 5 seconds");
            m_EnableMotorClientWaiting = false; 
        }

        rate.sleep();
    }
}

void GuiniverseNode::BarcodeCallback(const StringConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(barcode_mutex);
    shared_barcodes[msg->data]++;
}

void GuiniverseNode::TimerCallback()
{
    {
        std::lock_guard<std::mutex> lock(twist_mutex);
        m_TwistMessage = shared_twist;
    }
    m_TurtleTwistPublisher->publish(m_TwistMessage);

    {
        std::lock_guard<std::mutex> lock(gripper_mutex);
        m_GripperMessage = shared_gripper;
    }

}

void GuiniverseNode::EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) 
{
    RCLCPP_INFO(this->get_logger(), "Response: success=%s, message='%s'",
                response.get()->success ? "true" : "false",
                response.get()->message.c_str());

    m_EnableMotorClientWaiting = false;
}

void GuiniverseNode::SetMotorStatus(bool status)
{
    std::lock_guard<std::mutex> lock(shared_data_mutex);
    shared_data = status ? "Motors Enabled" : "Motors Disabled";
}
