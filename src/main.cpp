#include <guiniverse/ros2_imgui_integration.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>

// Shared data between threads
std::atomic<bool> running(true);
std::mutex data_mutex;
std::string shared_data = "Hello from ROS2!";

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // ROS2 on_shutdown handler to stop all threads
    rclcpp::on_shutdown([]() {
        running = false; // Notify both threads to stop
    });

    // Start ROS2 and GUI threads
    std::thread ros_thread(ros2_main_thread);
    std::thread gui_thread(imgui_thread);

    ros_thread.join();
    gui_thread.join();

    rclcpp::shutdown(); // Ensure ROS2 cleanup
    return 0;
}
