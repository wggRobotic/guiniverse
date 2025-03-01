#include <GL/glew.h>

#include <guiniverse/ControlGui/ControlGui.hpp>
#include <guiniverse/N10/N10.hpp>
#include <guiniverse/Idefix/Idefix.hpp>

ControlGui* gui_ptr;

int main(int argc, char **argv)
{
    if (!glfwInit()) return 1;
    glewInit();

    rclcpp::init(argc, argv);

    ControlGui gui(60);
    gui_ptr = &gui;

    rclcpp::on_shutdown([]() { gui_ptr->stop(); });

    gui.addController(std::make_shared<N10>());
    gui.addController(std::make_shared<Idefix>());

    gui.run();

    glfwTerminate();

    rclcpp::shutdown();

    return 0;
}
