#pragma once

#include <atomic>
#include <guiniverse/control_gui/console.hpp>
#include <guiniverse/control_gui/joystick_input.hpp>
#include <guiniverse/control_gui/robot_controller.hpp>
#include <vector>

#define NO_ROBOT_SELECTED -1

class ControlGui
{
public:
    ControlGui(int RobotControllerRefreshRate);

    void addController(std::shared_ptr<RobotController> Controller);

    void run();
    void stop();

private:
    void GuiFunction();
    void RobotControllerThreadFunction();

    std::atomic<bool> m_Running;

    std::vector<std::shared_ptr<RobotController>> m_Controllers;
    std::atomic<int> m_RobotSelected{ NO_ROBOT_SELECTED };
    std::atomic<bool> m_ChangeState{ false };

    JoystickInput m_JoystickInput;
    Console m_Console;

    rclcpp::Clock m_RosClock;

    rclcpp::Rate m_RosRate;
};
