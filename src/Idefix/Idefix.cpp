#include <guiniverse/Idefix/Idefix.hpp>

Idefix::Idefix() : RobotController("Idefix", "idefix")
{
    m_TwistPublisher = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
}

Idefix::~Idefix()
{
}


void Idefix::onFrame()
{

}

void Idefix::onGuiStart()
{

}

void Idefix::onGuiShutdown()
{

}

void Idefix::onGuiFrame(GLFWwindow* window, JoystickInput& input)
{

}