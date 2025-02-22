#pragma once

#include <rclcpp/rclcpp.hpp>

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <guiniverse/ControlGui/JoystickInput.hpp>

class RobotController : public rclcpp::Node
{
public:

    RobotController(const std::string& RobotName, const std::string& RosRobotName);

    virtual ~RobotController() = default;
    
    virtual void ImGuiPanels(GLFWwindow* window, JoystickInput& input) = 0;
    virtual void onFrame() = 0;

    const char* getRobotName() {return m_RobotName.c_str();};

protected:

    std::string m_RobotName;
    std::string m_RosRobotName;

};
