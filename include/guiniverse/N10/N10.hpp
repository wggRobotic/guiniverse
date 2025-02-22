#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>

#include <guiniverse/ImageSystem.hpp>

struct RoverWheel
{
    ImVec2 position = ImVec2(0, 0);

    float last_angle = 0;
    float last_speed = 0;

};

class N10 : public RobotController
{

public:
    N10();
    ~N10();

    void Init();

    void ImGuiPanels(GLFWwindow* window, JoystickInput& input) override;
    void onFrame() override;

private:

    std::shared_ptr<ImageSystem> m_ImageSystem;

    std::vector<RoverWheel> wheels;

    ImVec2 main_axes = ImVec2(0.f, 0.f);
    float scalar = 0.5f;
    float joystick_scalar = 0.5f;

    bool gas_button = false;

    bool enable_button = false;
    bool disable_button = false;
    bool physical_enable_button = false;
    bool physical_disable_button = false;

    bool gripper_mode_button = false;
    bool drive_mode_button = false;

    bool dog_walk_button = false;

    bool gripper_mode = false;

    ImVec2 linear_velocity = ImVec2(0.f, 0.f);
    float angular_velocity = 0.f;

    bool should_set_motor_status = false;
    bool motor_status = false;

};