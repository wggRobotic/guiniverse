#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Idefix : public RobotController
{
public:
    Idefix();
    ~Idefix();

    void onFrame() override;

    void onGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void onGuiStart() override;
    void onGuiShutdown() override;

private:

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    geometry_msgs::msg::Twist m_TwistMessage;    
    
};