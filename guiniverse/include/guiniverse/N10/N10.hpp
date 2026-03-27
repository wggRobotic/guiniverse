#pragma once

#include <guiniverse/ControlGui/RobotController.hpp>

#include <guiniverse/ImageSystem/ImageSystemBackendGST.hpp>
#include <guiniverse/DataCaptureSystem.hpp>

#include <atomic>
#include <mutex>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/set_bool.hpp>

struct RoverWheel
{
    float x = 0.f;
    float y = 0.f;
    float radius = 0.f;
    bool invert = false;

    float target_rpm = 0.f;
    float target_angle = 0.f;

    float last_rpm = 0.f;
    float last_angle = 0.f;
};

class N10 : public RobotController
{

public:
    N10();
    ~N10();

    void onFrame() override;
    void onStartup() override;
    void onShutdown() override;

    void onGuiFrame(GLFWwindow* window, JoystickInput& input) override;
    void onGuiStartup() override;
    void onGuiShutdown() override;

    void WheelsRPMFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void WheelsAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void GripperAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void GripperDistanceSensorCallback(const std_msgs::msg::Float32::SharedPtr msg);

    void VoltagePowerManagementCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void VoltageAdatpterCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void WheelsEnabledCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg);

    void EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response);

private:

    void addWheel(float x, float y, float radius, bool invert);

    bool calculateGripperAngles(float x, float y, float ground_angle, float* result_angles);

    std::shared_ptr<ImageSystem> m_ImageSystem;
    std::shared_ptr<ImageSystemBackendGST> m_ImageSystemBackendGST;

    std::shared_ptr<DataCaptureSystem> m_DataCaptureSystem;

    std::mutex m_WheelsMutex;
    std::vector<RoverWheel> m_Wheels;

    std::mutex m_GripperMutex;
    struct
    {
        float segments[3] = {0.12f, 0.115f, 0.11f};

        float drive_position_angles[4] = {1.4f, 1.4f, 0.6f, 0.f };

        float target_x = 0.22f;
        float target_y = 0.0f;
        float target_ground_angle = 0.f;

        float current_x = 0.22f;
        float current_y = 0.0f;
        float current_ground_angle = 0.f;
        float current_gripper_state = 0.f;

        float feedback_angles[4] = {0.f, 0.f, 0.f, 0.f};

        bool inrange = true;
        bool ready = false;
    } m_Gripper;

    std::atomic<float> m_GripperDistanceSensorDistance{0.f};
    std::atomic<float> m_VoltagePowerManagement{0.f};
    std::atomic<float> m_VoltageAdapter{0.f};
    std::atomic<bool> m_Enabled{false};

    std::mutex m_InputMutex;

    struct 
    {
        struct 
        {
            float main_axis_x = 0.f;
            float main_axis_y = 0.f;

            float scalar_axis = 0.5f;
            float scalar_axis_joystick = 0.5f;

            bool gas_button = false;
            bool dog_walk_button = false;

            bool enable_button = false;
            bool disable_button = false;
            bool enable_button_physical = false;
            bool disable_button_physical = false;
        } drive;

        struct
        {
            float up_axis = 0.f;
            float forward_axis = 0.f;
            float ground_angle_axis = 0.f;

            float gripper_state = 0.5f;
            float gripper_state_joystick = 0.5f;

            bool send_angles = false;
        } gripper;

    } m_Input;

    std::atomic<bool> m_GripperMode{false};

    //ros2

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

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TurtleTwistPublisher;
    geometry_msgs::msg::Twist m_TurtleTwistMessage;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_EnableMotorClient;
    bool m_EnableMotorClientWaiting = false;
    int m_EnableMotorClientTimeSent = 0;

};