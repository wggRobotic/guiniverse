#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>

class Idefix : public RobotController
{
private:

public:
    Idefix();
    ~Idefix();

    void ImGuiPanels(GLFWwindow* window, JoystickInput& input) override;
    void onFrame() override;
};