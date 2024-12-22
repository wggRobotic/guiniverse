#include <guiniverse/ros2_imgui_integration.hpp>

// Shared data between threads
std::atomic<bool> running(true);
std::mutex data_mutex;
std::string shared_data = "Hello from ROS2!";


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::thread ros_thread(ros2_main_thread);
    std::thread gui_thread(imgui_thread);

    ros_thread.join();
    gui_thread.join();

    rclcpp::shutdown();
    return 0;
}