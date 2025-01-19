#include <guiniverse/imgui.hpp>
#include <guiniverse/ros.hpp>
#include <guiniverse/shared_data.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::on_shutdown([]()
                        { running = false; });

    std::thread com_thread(ros_thread);
    std::thread gui_thread(imgui_thread);

    com_thread.join();
    gui_thread.join();

    rclcpp::shutdown();
    return 0;
}
