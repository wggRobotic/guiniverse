#include <guiniverse/ros2_imgui_integration.hpp>

using namespace std::chrono_literals;

class GuiniverseNode : public rclcpp::Node {
public:
    GuiniverseNode() : Node("guiniverse"), count_(0), running_(true) {
        // Initialize timer
        timer_ = this->create_wall_timer(
            500ms, std::bind(&GuiniverseNode::timer_callback, this)
        );
    }

    // Start the ROS2 main thread
    void run() {
        rclcpp::Rate rate(10);
        while (rclcpp::ok() && running_) {
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                shared_data_ = "ROS2 is running: " + std::to_string(this->now().seconds());
            }
            rclcpp::spin_some(shared_from_this());
            rate.sleep();
        }
    }

    // Stop the node safely
    void stop() {
        running_ = false;
    }

private:
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Timer callback called. Count: %d", count_++);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex data_mutex_;
    std::string shared_data_;
    std::atomic<bool> running_;
    int count_;
};

// ROS2 main thread
void ros2_main_thread() {
    auto node = std::make_shared<GuiniverseNode>();

    // Run the ROS2 main loop in the node
    node->run();

    rclcpp::shutdown();
}
