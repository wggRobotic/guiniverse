#pragma once

#include <atomic>
#include <geometry_msgs/msg/twist.hpp>
#include <guiniverse/data_capture_system.hpp>
#include <guiniverse/image_system/image_system_backend_gst.hpp>
#include <guiniverse/robot_controller.hpp>
#include <mutex>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

struct RoverWheel final
{
    float X = 0.f;
    float Y = 0.f;
    float Radius = 0.f;
    bool Invert = false;

    float TargetRPM = 0.f;
    float TargetAngle = 0.f;

    float LastRPM = 0.f;
    float LastAngle = 0.f;
};

class N10 final : public RobotController
{
public:
    N10();
    ~N10();

    void OnFrame() override;
    void OnStartup() override;
    void OnShutdown() override;

    void OnGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void OnGuiStartup() override;
    void OnGuiShutdown() override;

    void WheelsRPMFeedbackCallback(const std_msgs::msg::Float32MultiArray::UniquePtr msg);
    void WheelsAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::UniquePtr msg);
    void GripperAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::UniquePtr msg);
    void GripperDistanceSensorCallback(const std_msgs::msg::Float32::UniquePtr msg);

    void VoltagePowerManagementCallback(const std_msgs::msg::Float32::UniquePtr msg);
    void VoltageAdapterCallback(const std_msgs::msg::Float32::UniquePtr msg);
    void WheelsEnabledCallback(const std_msgs::msg::ByteMultiArray::UniquePtr msg);

    void EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response);

private:
    void AddWheel(float x, float y, float radius, bool invert);

    bool CalculateGripperAngles(float x, float y, float ground_angle, float* result_angles);

    std::shared_ptr<ImageSystem> m_ImageSystem;
    std::shared_ptr<ImageSystemBackendGST> m_ImageSystemBackendGST;

    std::shared_ptr<DataCaptureSystem> m_DataCaptureSystem;

    std::mutex m_WheelsMutex;
    std::vector<RoverWheel> m_Wheels;

    std::mutex m_GripperMutex;
    struct
    {
        float Segments[3] = { 0.12f, 0.115f, 0.11f };

        float DrivePositionAngles[4] = { 1.4f, 1.4f, 0.6f, 0.f };

        float TargetX = 0.22f;
        float TargetY = 0.0f;
        float TargetGroundAngle = 0.f;

        float CurrentX = 0.22f;
        float CurrentY = 0.0f;
        float CurrentGroundAngle = 0.f;
        float CurrentGripperState = 0.f;

        float FeedbackAngles[4] = { 0.f, 0.f, 0.f, 0.f };

        bool InRange = true;
        bool Ready = false;
    } m_Gripper;

    std::atomic<float> m_GripperDistanceSensorDistance{ 0.f };
    std::atomic<float> m_VoltagePowerManagement{ 0.f };
    std::atomic<float> m_VoltageAdapter{ 0.f };
    std::atomic<bool> m_Enabled{ false };

    std::mutex m_InputMutex;

    struct
    {
        struct
        {
            float MainAxisX = 0.f;
            float MainAxisY = 0.f;

            float ScalarAxis = 0.5f;
            float ScalarAxisJoystick = 0.5f;

            bool GasButton = false;
            bool DogWalkButton = false;

            bool EnableButton = false;
            bool DisableButton = false;
            bool EnableButtonPhysical = false;
            bool DisableButtonPhysical = false;
        } Drive;

        struct
        {
            float UpAxis = 0.f;
            float ForwardAxis = 0.f;
            float GroundAngleAxis = 0.f;

            float GripperState = 0.5f;
            float GripperStateJoystick = 0.5f;

            bool SendAngles = false;
        } Gripper;

    } m_Input;

    std::atomic<bool> m_GripperMode{ false };

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsRPMFeedbackSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsAngleFeedbackSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_GripperAngleFeedbackSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_GripperDistanceSensorSubscriber;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_VoltagePowerManagementSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_VoltageAdapterSubscriber;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr m_WheelsEnabledSubscriber;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsRPMPublisher;
    std_msgs::msg::Float32MultiArray m_WheelsRPMMessage;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsAnglePublisher;
    std_msgs::msg::Float32MultiArray m_WheelsAngleMessage;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_GripperAnglePublisher;
    std_msgs::msg::Float32MultiArray m_GripperAngleMessage;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_EnableMotorClient;
    bool m_EnableMotorClientWaiting = false;
    int m_EnableMotorClientTimeSent = 0;
};
