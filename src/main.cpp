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

    std::shared_ptr<N10> n10 = std::make_shared<N10>();
    n10->initImageSystem();

    n10->addImageTopic("front/color");
    n10->addImageTopic("rear/color");
    n10->addImageTopic("gripper/color");

    n10->addWheel( 0.152f,  0.105f, 0.05f, false);
    n10->addWheel( 0.152f, -0.105f, 0.05f, true );
    n10->addWheel( 0.0,     0.105f, 0.05f, false);
    n10->addWheel( 0.0f,   -0.105f, 0.05f, true );
    n10->addWheel(-0.152f,  0.105f, 0.05f, false);
    n10->addWheel(-0.152f, -0.105f, 0.05f, true );

    std::shared_ptr<Idefix> idefix = std::make_shared<Idefix>();

    gui.addController(n10);
    gui.addController(idefix);

    gui.run();

    glfwTerminate();

    rclcpp::shutdown();

    return 0;
}
