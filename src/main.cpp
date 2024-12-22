#include "guiniverse/ros2_imgui_integration.hpp"

// Shared data between threads
std::atomic<bool> running(true);
std::mutex data_mutex;
std::string shared_data = "Hello from ROS2!";

// ROS2 main thread
void ros2_main_thread() {
    auto node = rclcpp::Node::make_shared("ros2_thread");
    rclcpp::Rate rate(10);

    while (rclcpp::ok() && running) {
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            shared_data = "ROS2 is running: " + std::to_string(rclcpp::Clock().now().seconds());
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::thread ros_thread(ros2_main_thread);
    std::thread gui_thread(imgui_thread);

    ros_thread.join();
    gui_thread.join();

    rclcpp::shutdown();
    return 0;
}