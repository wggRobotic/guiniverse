#include <guiniverse/node.hpp>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

std::atomic<bool> running{true};

void ros2_main_thread();
void imgui_thread();

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::on_shutdown([]()
                        { running = false; });

    std::thread ros_thread(ros2_main_thread);
    std::thread gui_thread(imgui_thread);

    ros_thread.join();
    gui_thread.join();

    rclcpp::shutdown();
    return 0;
}

void ros2_main_thread()
{
    auto node = std::make_shared<GuiniverseNode>();
    image_transport::ImageTransport image_transport(node);
    node->SetupWithImageTransport(image_transport);
    node->run();
}
