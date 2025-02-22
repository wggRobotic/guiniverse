#pragma once

#include <vector>
#include <atomic>

#include <guiniverse/ControlGui/RobotController.hpp>

#include <guiniverse/ControlGui/JoystickInput.hpp>
#include <guiniverse/ControlGui/Console.hpp>

#define NO_ROBOT_SELECTED -1

class ControlGui {

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
    int32_t m_RobotSelected = NO_ROBOT_SELECTED;


    JoystickInput m_JoystickInput;
    Console m_Console;

    rclcpp::Clock m_RosClock;
    int m_RobotControllerRefreshRate;

};