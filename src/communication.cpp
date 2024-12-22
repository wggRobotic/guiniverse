#include <guiniverse/ros2_imgui_integration.hpp>

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
