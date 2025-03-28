#pragma once

#include <rclcpp/rclcpp.hpp>

#include <guiniverse/ControlGui/JoystickInput.hpp>

#include <GLFW/glfw3.h>
#include <imgui.h>

class RobotController
{
public:

    friend class ControlGui;

    RobotController(const std::string& RobotName, const std::string& RosRobotName) 
        : m_RobotName(RobotName), m_RosRobotName(RosRobotName) {};

    virtual ~RobotController() = default;
    

    virtual void onFrame() = 0;
    virtual void onStartup() = 0;
    virtual void onShutdown() = 0;
    
    virtual void onGuiFrame(GLFWwindow* window, JoystickInput& input) = 0;
    virtual void onGuiStartup() = 0;
    virtual void onGuiShutdown() = 0;

    const char* getRobotName() {return m_RobotName.c_str();};

protected:

    std::string m_RobotName;
    std::string m_RosRobotName;
    rclcpp::Node::SharedPtr node;

private:

    void _onStartup() {
        node = std::make_shared<rclcpp::Node>(m_RosRobotName + "_controller_node", "/" + m_RosRobotName);
        onStartup();
    };

    void _onShutdown() {
        onShutdown();
        node.reset();
    };

};
