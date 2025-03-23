#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>

#include <guiniverse/ImageSystem/ImageSystemBackendGST.hpp>
#include <guiniverse/DataCaptureSystem.hpp>

#include <atomic>
#include <mutex>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>

class Idefix : public RobotController
{
public:
    Idefix();
    ~Idefix();

    void onFrame() override;
    void onStartup() override;
    void onShutdown() override;

    void onGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void onGuiStartup() override;
    void onGuiShutdown() override;

    void IMUCallback(const std_msgs::msg::Float32::SharedPtr msg);

private:

    std::shared_ptr<ImageSystem> m_ImageSystem;
    std::shared_ptr<ImageSystemBackendGST> m_ImageSystemBackendGST;

    std::shared_ptr<DataCaptureSystem> m_DataCaptureSystem;

    std::mutex m_InputMutex;

    struct {
        ImVec2 main_axes = ImVec2(0.f, 0.f);
        float scalar = 0.5f;
        float joystick_scalar = 0.5f;

        float roll = 0.f;
        float pitch = 0.f;
        float yaw = 0.f;

        bool gas_button = false;
    } m_Input;
    
    std::atomic<float> m_IMUAngle{0.f};

    //ros2

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_IMUSubscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    geometry_msgs::msg::Twist m_TwistMessage;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TurtleTwistPublisher;
    geometry_msgs::msg::Twist m_TurtleTwistMessage;
    
};