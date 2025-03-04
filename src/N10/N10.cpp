#include <guiniverse/N10/N10.hpp>

N10::N10() : RobotController("N10", "n10")
{

    m_WheelsRPMFeedbackSubscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>("wheels/rpm/feedback", 10, std::bind(&N10::WheelsRPMFeedbackCallback, this, std::placeholders::_1));
    m_WheelsAngleFeedbackSubscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>("wheels/angle/feedback", 10, std::bind(&N10::WheelsServoFeedbackCallback, this, std::placeholders::_1));
    m_GripperFeedbackSubscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>("gripper/feedback", 10, std::bind(&N10::GripperServoFeedbackCallback, this, std::placeholders::_1));

    m_WheelsRPMPublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("wheels/rpm/cmd", 10);
    m_WheelsAnglePublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("wheels/angle/cmd", 10);
    m_GripperPublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("gripper/cmd", 10);

    m_TurtleTwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_EnableMotorClient = node->create_client<std_srvs::srv::SetBool>("wheels/enable");

    m_Wheels.reserve(6);

    addWheel( 0.152f,  0.105f, 0.05f, false);
    addWheel( 0.152f, -0.105f, 0.05f, true );
    addWheel( 0.0,     0.105f, 0.05f, false);
    addWheel( 0.0f,   -0.105f, 0.05f, true );
    addWheel(-0.152f,  0.105f, 0.05f, false);
    addWheel(-0.152f, -0.105f, 0.05f, true );

    m_ImageSystem = std::make_shared<ImageSystem>(node);

    m_ImageSystem->addTopic("front/cam/color");
    m_ImageSystem->addTopic("rear/cam/color");
    m_ImageSystem->addTopic("gripper/cam/color");

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);

    m_DataCaptureSystem->addSection("Barcodes", "barcode");
}

N10::~N10()
{
}

void N10::addWheel(float x, float y, float radius, bool invert) 
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    RoverWheel wheel = {x, y, radius, invert, 0.f, 0.f, 0.f, 0.f};

    m_Wheels.push_back(wheel);
}

void N10::WheelsRPMFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    if (msg->data.size() == m_Wheels.size()) for (int i = 0; i < m_Wheels.size(); i++)
        m_Wheels.at(i).last_rpm = msg->data.at(i);
    
}

void N10::WheelsServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    if (msg->data.size() == m_Wheels.size()) for (int i = 0; i < m_Wheels.size(); i++)
        m_Wheels.at(i).last_angle = msg->data.at(i);
}

void N10::GripperServoFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_GripperMutex);
}

void N10::EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) 
{
    RCLCPP_INFO(node->get_logger(), "Response: success=%s, message='%s'",
                response.get()->success ? "true" : "false",
                response.get()->message.c_str());

    m_EnableMotorClientWaiting = false;
}