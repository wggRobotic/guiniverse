#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>
#include <guiniverse/DataCaptureSystem.hpp>

#include <atomic>
#include <mutex>
#include <vector>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "edu_robot/srv/set_mode.hpp"
#include <geometry_msgs/msg/twist.hpp>

struct ExplorerWheel
{
    float x = 0.f;
    float y = 0.f;
    float radius = 0.f;
    bool invert = false;

    float target_rpm = 0.f;
    float last_rpm = 0.f;
};

class NoName : public RobotController
{
public:
    NoName();
    ~NoName();

    void onFrame() override;

    void onGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void onGuiStartup() override;
    void onGuiShutdown() override;

    void SetModeClientCallback(rclcpp::Client<edu_robot::srv::SetMode>::SharedFuture response);

private:

    void addWheel(float x, float y, float radius, bool invert);

    std::shared_ptr<DataCaptureSystem> m_DataCaptureSystem;

    std::mutex m_WheelsMutex;
    std::vector<ExplorerWheel> m_Wheels;

    std::mutex m_InputMutex;

    struct {
        ImVec2 main_axes = ImVec2(0.f, 0.f);
        float scalar = 0.5f;
        float joystick_scalar = 0.5f;

        bool gas_button = false;

        bool enable_button = false;
        bool disable_button = false;
        bool enable_button_physical = false;
        bool disable_button_physical = false;
    } m_Input;
    
    //ros2

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_RPMPublisher;
    std_msgs::msg::Float32MultiArray m_RPMMessage;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TurtleTwistPublisher;
    geometry_msgs::msg::Twist m_TurtleTwistMessage;

    rclcpp::Client<edu_robot::srv::SetMode>::SharedPtr m_SetModeClient;
    bool m_SetModeClientWaiting = false;
    int m_SetModeClientTimeSent = 0;  
};