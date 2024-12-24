#include <guiniverse/node.hpp>
using namespace std::chrono_literals;

// Shared data between threads
std::atomic<bool> running{true};
std::mutex data_mutex;
std::string shared_data = "Hello from ROS2!";

void ros2_main_thread();
void imgui_thread();

int main(int argc, char **argv)
{
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

void ros2_main_thread()
{
    // Create a ROS2 node instance
    auto node = std::make_shared<GuiniverseNode>();

    // Example: Setup ImageTransport if required
    image_transport::ImageTransport image_transport(node);
    //node->SetupWithImageTransport(image_transport);

    // Run the node's main logic
    node->run();
}
