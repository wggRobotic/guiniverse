#pragma once

#include <GLFW/glfw3.h>
#include <guiniverse/control_gui/joystick_input.hpp>
#include <imgui.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

class RobotController
{
public:
    friend class ControlGui;

    RobotController(const std::string& robot_name, const std::string& ros_robot_name);

    virtual ~RobotController() = default;

    virtual void OnFrame() = 0;
    virtual void OnStartup() = 0;
    virtual void OnShutdown() = 0;

    virtual void OnGuiFrame(GLFWwindow* window, JoystickInput& input) = 0;
    virtual void OnGuiStartup() = 0;
    virtual void OnGuiShutdown() = 0;

    const char* GetRobotNameC() const;

    const std::string& GetRobotName() const;
    const std::string& GetRosRobotName() const;

protected:
    rclcpp::Node::SharedPtr GetNode() const;

private:
    void Start();
    void Shutdown();

private:
    std::string m_RobotName;
    std::string m_RosRobotName;
    rclcpp::Node::SharedPtr m_Node;
};
