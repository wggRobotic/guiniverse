#pragma once

#include <guiniverse/ImageSystem.hpp>
#include <guiniverse/ControlGui/RobotController.hpp>

#include <atomic>
#include <mutex>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
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

    void initImageSystem();
    void addWheel(float x, float y, float radius, bool invert);
    void ImGuiPanels(GLFWwindow* window, JoystickInput& input) override;
    void onFrame() override;

    void BarcodeCallback(const std_msgs::msg::String::SharedPtr msg);

    void WheelsRPMFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void WheelsServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void GripperServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response);

private:

    std::shared_ptr<ImageSystem> m_ImageSystem;

    std::mutex m_WheelsMutex;
    std::vector<RoverWheel> m_Wheels;

    std::mutex m_GripperMutex;

    std::mutex m_InputMutex;

    struct {
        ImVec2 main_axes = ImVec2(0.f, 0.f);
        float scalar = 0.5f;
        float joystick_scalar = 0.5f;

        bool gas_button = false;
        bool dog_walk_button = false;
        
        bool enable_button = false;
        bool disable_button = false;
        bool physical_enable_button = false;
        bool physical_disable_button = false;

        ImVec2 linear_velocity = ImVec2(0.f, 0.f);
        float angular_velocity = 0.f;

        bool should_set_motor_status = false;
        bool motor_status = false;

    } m_Input;

    std::atomic<bool> m_GripperMode{false};

    std::mutex m_BarcodesMutex;
    std::map<std::string, size_t> m_Barcodes;

    //ros2

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_BarcodeSubscriber;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsRPMFeedbackSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsServoFeedbackSubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr m_GripperServoFeedbackSubscriber;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsRPMPublisher;
    std_msgs::msg::Float32MultiArray m_WheelsRPMMessage;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_WheelsServoPublisher;
    std_msgs::msg::Float32MultiArray m_WheelsServoMessage;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr m_GripperServoPublisher;
    std_msgs::msg::Float32MultiArray m_GripperServoMessage;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_TurtleTwistPublisher;
    geometry_msgs::msg::Twist m_TurtleTwistMessage;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_EnableMotorClient;
    bool m_EnableMotorClientWaiting = false;
    int m_EnableMotorClientTimeSent = 0;

};