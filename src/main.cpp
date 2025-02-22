// #include <guiniverse/imgui.hpp>
// #include <guiniverse/ros.hpp>
// #include <guiniverse/shared_data.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <guiniverse/ControlGui/ControlGui.hpp>
#include <guiniverse/N10/N10.hpp>
#include <guiniverse/Idefix/Idefix.hpp>

ControlGui gui(60);

int main(int argc, char **argv)
{

    if (!glfwInit()) return 1;
    glewInit();

    rclcpp::init(argc, argv);

    rclcpp::on_shutdown([]() { gui.stop(); });

    std::shared_ptr<N10> n10 = std::make_shared<N10>();
    n10->Init();
    std::shared_ptr<RobotController> idefix = std::make_shared<Idefix>();

    gui.addController(n10);
    gui.addController(idefix);

    gui.run();

    rclcpp::shutdown();

    glfwTerminate();

    return 0;
}
