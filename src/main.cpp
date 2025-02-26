#include <GL/glew.h>

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
    n10->initImageSystem();

    n10->addWheel( 0.152f,  0.105f, 0.05f, false);
    n10->addWheel( 0.152f, -0.105f, 0.05f, true );
    n10->addWheel( 0.0,     0.105f, 0.05f, false);
    n10->addWheel( 0.0f,   -0.105f, 0.05f, true );
    n10->addWheel(-0.152f,  0.105f, 0.05f, false);
    n10->addWheel(-0.152f, -0.105f, 0.05f, true );

    std::shared_ptr<RobotController> idefix = std::make_shared<Idefix>();

    gui.addController(n10);
    gui.addController(idefix);

    gui.run();

    rclcpp::shutdown();

    glfwTerminate();

    return 0;
}
