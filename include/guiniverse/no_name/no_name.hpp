#pragma once

#include <edu_robot/srv/set_mode.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <guiniverse/data_capture_system.hpp>
#include <guiniverse/image_system/image_system_backend_gst.hpp>
#include <guiniverse/robot_controller.hpp>
#include <mutex>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

struct ExplorerWheel final
{
    float X = 0.f;
    float Y = 0.f;
    float Radius = 0.f;
    bool Invert = false;

    float TargetRPM = 0.f;
    float LastRPM = 0.f;
};

class NoName final : public RobotController
{
public:
    NoName();
    ~NoName();

    void OnFrame() override;
    void OnStartup() override;
    void OnShutdown() override;

    void OnGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void OnGuiStartup() override;
    void OnGuiShutdown() override;

    void SetModeClientCallback(rclcpp::Client<edu_robot::srv::SetMode>::SharedFuture response);

private:
    void AddWheel(float x, float y, float radius, bool invert);

    std::shared_ptr<ImageSystem> m_ImageSystem;
    std::shared_ptr<ImageSystemBackendGST> m_ImageSystemBackendGST;

    std::shared_ptr<DataCaptureSystem> m_DataCaptureSystem;

    std::mutex m_WheelsMutex;
    std::vector<ExplorerWheel> m_Wheels;

    std::mutex m_InputMutex;

    struct
    {
        ImVec2 MainAxes = ImVec2(0.f, 0.f);
        float Scalar = 0.5f;
        float JoystickScalar = 0.5f;

        bool GasButton = false;

        bool EnableButton = false;
        bool DisableButton = false;
        bool EnableButtonPhysical = false;
        bool DisableButtonPhysical = false;
    } m_Input;

    // ros2

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_RPMPublisher;
    std_msgs::msg::Float32MultiArray m_RPMMessage;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TurtleTwistPublisher;
    geometry_msgs::msg::Twist m_TurtleTwistMessage;

    rclcpp::Client<edu_robot::srv::SetMode>::SharedPtr m_SetModeClient;
    bool m_SetModeClientWaiting = false;
    int m_SetModeClientTimeSent = 0;
};
