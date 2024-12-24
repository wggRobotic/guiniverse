#include <guiniverse/ros2_imgui_integration.hpp>
#include <guiniverse/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace std::chrono_literals;

GuiniverseNode::GuiniverseNode()
    : Node("guiniverse"), count_(0), running_(true)
{
    // Initialize timer
    m_Timer = this->create_wall_timer(
        500ms, std::bind(&GuiniverseNode::TimerCallback, this));

    // Initialize publishers
    m_TwistPublisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_GripperPublisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("gripper", 10);

    // Initialize subscribers
    m_BarcodeSubscriber = this->create_subscription<std_msgs::msg::String>(
        "barcode", 10, std::bind(&GuiniverseNode::BarcodeCallback, this, std::placeholders::_1));

    // Initialize service client
    m_EnableMotorClient = this->create_client<std_srvs::srv::SetBool>("enable_motor");
}

void GuiniverseNode::run()
{
    rclcpp::Rate rate(10); // 10 Hz loop rate
    while (rclcpp::ok() && running_)
    {
        {
            // Lock the mutex to safely access shared data
            std::lock_guard<std::mutex> lock(data_mutex);
            shared_data = "ROS2 is running: " + std::to_string(this->now().seconds());
        }

        // Execute ROS2 callbacks
        rclcpp::spin_some(shared_from_this());
        rate.sleep();
    }
}

void GuiniverseNode::BarcodeCallback(const StringConstPtr &msg)
{
    // Update barcode information in a thread-safe way
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        RCLCPP_INFO(this->get_logger(), "Received barcode: %s", msg->data.c_str());
        m_Barcodes[msg->data]++;
    }
}

void GuiniverseNode::TimerCallback()
{
    // Periodically execute logic for motor control or robot updates
    RCLCPP_INFO(this->get_logger(), "Timer callback called. Count: %d", count_++);

    // Example: Publish a simple Twist message
    m_TwistMessage.linear.x = 0.1;
    m_TwistPublisher->publish(m_TwistMessage);

    // Check if motor status needs to be updated
    if (m_ShouldSetMotorStatusTrue || m_ShouldSetMotorStatusFalse)
    {
        SetMotorStatus(m_ShouldSetMotorStatusTrue);
    }
}

void GuiniverseNode::SetMotorStatus(bool status)
{
    // Prevent simultaneous motor status updates
    if (m_SetMotorStatusLock)
    {
        return;
    }

    m_SetMotorStatusLock = true;

    // Create service request to change motor status
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = status;

    // Send service request and process the response
    auto future = m_EnableMotorClient->async_send_request(request);
    future.wait();

    auto response = future.get();
    if (response->success)
    {
        RCLCPP_INFO(this->get_logger(), "Motor status set to: %s", status ? "true" : "false");
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Failed to set motor status");
    }

    m_SetMotorStatusLock = false;
}
