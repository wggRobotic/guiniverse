#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>

class Idefix : public RobotController
{
private:

public:
    Idefix();
    ~Idefix();

    void onFrame() override;

    void onGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void onGuiStart() override;
    void onGuiShutdown() override;
    
};