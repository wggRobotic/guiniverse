#pragma once

#include <atomic>
#include <guiniverse/control_gui/console.hpp>
#include <guiniverse/control_gui/joystick_input.hpp>
#include <guiniverse/robot_controller.hpp>
#include <vector>

constexpr auto NO_ROBOT_SELECTED = -1;

class ControlGui final
{
public:
    ControlGui(int robot_controller_refresh_rate);

    void AddController(std::shared_ptr<RobotController> controller);

    void Run();
    void Stop();

protected:
    void GuiFunction();
    void RobotControllerThreadFunction();

private:
    std::atomic<bool> m_Running;

    std::vector<std::shared_ptr<RobotController>> m_Controllers;
    std::atomic<int> m_RobotSelected = NO_ROBOT_SELECTED;
    std::atomic<bool> m_ChangeState = false;

    JoystickInput m_JoystickInput;
    Console m_Console;

    rclcpp::Clock m_RosClock;

    rclcpp::Rate m_RosRate;
};
