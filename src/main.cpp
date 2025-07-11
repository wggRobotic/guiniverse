#include <GL/glew.h>
#include <gst/gst.h>
#include <guiniverse/control_gui/control_gui.hpp>
#include <guiniverse/idefix/idefix.hpp>
#include <guiniverse/n10/n10.hpp>
#include <guiniverse/no_name/no_name.hpp>

ControlGui* gui_ptr;

int main(int argc, char** argv)
{
    if (!glfwInit())
        return 1;
    glewInit();

    rclcpp::init(argc, argv);

    gst_init(NULL, NULL);

    ControlGui gui(60);
    gui_ptr = &gui;

    rclcpp::on_shutdown([]() { gui_ptr->Stop(); });

    gui.AddController(std::make_shared<N10>());
    gui.AddController(std::make_shared<Idefix>());
    gui.AddController(std::make_shared<NoName>());

    gui.Run();

    rclcpp::shutdown();

    glfwTerminate();

    return 0;
}
