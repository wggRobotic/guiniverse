#pragma once

#include <guiniverse/ImageSystem.hpp>
#include <guiniverse/ControlGui/RobotController.hpp>
#include <guiniverse/DataCaptureSystem.hpp>

#include <atomic>
#include <mutex>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "edu_robot/srv/set_mode.hpp"
#include <geometry_msgs/msg/twist.hpp>

class NoName : public RobotController
{
public:
    NoName();
    ~NoName();

    void onFrame() override;

    void onGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void onGuiStart() override;
    void onGuiShutdown() override;

    void SetModeClientCallback(rclcpp::Client<edu_robot::srv::SetMode>::SharedFuture response);

private:

    std::shared_ptr<ImageSystem> m_ImageSystem;
    std::shared_ptr<DataCaptureSystem> m_DataCaptureSystem;

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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    geometry_msgs::msg::Twist m_TwistMessage;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TurtleTwistPublisher;
    geometry_msgs::msg::Twist m_TurtleTwistMessage;

    rclcpp::Client<edu_robot::srv::SetMode>::SharedPtr m_SetModeClient;
    bool m_SetModeClientWaiting = false;
    int m_SetModeClientTimeSent = 0;  
};