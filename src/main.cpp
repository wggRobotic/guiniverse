#include <GL/glew.h>

#include <guiniverse/ControlGui/ControlGui.hpp>

#include <guiniverse/N10/N10.hpp>
#include <guiniverse/Idefix/Idefix.hpp>
#include <guiniverse/NoName/NoName.hpp>

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
    gui.addController(std::make_shared<NoName>());

    gui.run();

    rclcpp::shutdown();

    glfwTerminate();

    return 0;
}
