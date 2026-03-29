#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>

#include <guiniverse/DataCaptureSystem.hpp>
#include <guiniverse/ImageSystem/ImageSystemBackendGST.hpp>

#include <atomic>
#include <mutex>
#include "geometry_msgs/msg/twist.hpp"

#include "imgui.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

struct arm_joint
{
    int index;
    double value;
};

class Quac : public RobotController
{
public:
    Quac();
    ~Quac();

    void onFrame() override;
    void onStartup() override;
    void onShutdown() override;

    void onGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void onGuiStartup() override;
    void onGuiShutdown() override;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
    std::shared_ptr<DataCaptureSystem> m_DataCaptureSystem;

    std::shared_ptr<ImageSystem> m_ImageSystem;
    std::shared_ptr<ImageSystemBackendGST> m_ImageSystemBackendGST;

    std::mutex m_InputMutex;

    struct {
        ImVec2 main_axes = ImVec2(0.f, 0.f);
        float scalar = 0.5f;
        float joystick_scalar = 0.5f;

        bool gas_button = false;
        bool publish_cmd = true;
    } m_Input;

    struct
    {
        struct arm_joint joints[3];

        ImVec2 target_pose;
        bool publish_pose;

        std::mutex mutex;
    } m_Arm;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TwistPublisher;
    geometry_msgs::msg::Twist m_TwistMessage;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr m_JointStatesSubscriber;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_ArmPosePublisher;
    geometry_msgs::msg::Pose m_ArmPoseMessage;
};