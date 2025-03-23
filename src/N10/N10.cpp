#include <guiniverse/N10/N10.hpp>

N10::N10() : RobotController("N10", "n10")
{

    m_GripperAngleMessage.data.resize(4);
    m_GripperAngleMessage.data[0] = m_Gripper.drive_position_angles[0];
    m_GripperAngleMessage.data[1] = m_Gripper.drive_position_angles[1];
    m_GripperAngleMessage.data[2] = m_Gripper.drive_position_angles[2];
    m_GripperAngleMessage.data[3] = m_Gripper.drive_position_angles[3];

    m_Wheels.reserve(6);

    addWheel( 0.152f,  0.105f, 0.055f, true ); //0
    addWheel( 0.152f, -0.105f, 0.055f, false); //1
    addWheel( 0.0,     0.105f, 0.052f, true);  //2
    addWheel( 0.0f,   -0.105f, 0.052f, false); //3
    addWheel(-0.152f,  0.105f, 0.055f, true ); //4
    addWheel(-0.152f, -0.105f, 0.055f, false); //5
}

N10::~N10()
{
}

void N10::onStartup() 
{
    for (int i = 0; i < m_Wheels.size(); i++)
    {
        m_Wheels[i].target_rpm = 0.f;
        m_Wheels[i].target_angle = 0.f;
        m_Wheels[i].last_rpm = 0.f;
        m_Wheels[i].last_angle = 0.f;
    }

    m_EnableMotorClientWaiting = false;

    m_WheelsRPMFeedbackSubscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>("wheels/rpm/feedback", 10, std::bind(&N10::WheelsRPMFeedbackCallback, this, std::placeholders::_1));
    m_WheelsAngleFeedbackSubscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>("wheels/angle/feedback", 10, std::bind(&N10::WheelsAngleFeedbackCallback, this, std::placeholders::_1));
    m_GripperAngleFeedbackSubscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>("gripper/angle/feedback", 10, std::bind(&N10::GripperAngleFeedbackCallback, this, std::placeholders::_1));
    m_GripperDistanceSensorSubscriber = node->create_subscription<std_msgs::msg::Float32>("gripper/distance_sensor", 10, std::bind(&N10::GripperDistanceSensorCallback, this, std::placeholders::_1));

    m_VoltagePowerManagementSubscriber = node->create_subscription<std_msgs::msg::Float32>("voltagePwrMgmt", 10, std::bind(&N10::VoltagePowerManagementCallback, this, std::placeholders::_1));
    m_VoltageAdapterSubscriber = node->create_subscription<std_msgs::msg::Float32>("voltageAdapter", 10, std::bind(&N10::VoltageAdatpterCallback, this, std::placeholders::_1));;
    m_WheelsEnabledSubscriber = node->create_subscription<std_msgs::msg::ByteMultiArray>("wheels/enabled", 10, std::bind(&N10::WheelsEnabledCallback, this, std::placeholders::_1));


    m_WheelsRPMPublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("wheels/rpm/cmd", 10);
    m_WheelsAnglePublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("wheels/angle/cmd", 10);
    m_GripperAnglePublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("gripper/angle/cmd", 10);

    m_TurtleTwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_EnableMotorClient = node->create_client<std_srvs::srv::SetBool>("wheels/enable");


    m_ImageSystem = std::make_shared<ImageSystem>(node);

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->addSink(5000, ImageSystemAddOn_QRCode);
    m_ImageSystemBackendGST->addSink(5001, ImageSystemAddOn_QRCode);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);
    m_DataCaptureSystem->addSection("QRCodes", "qrcode");
    m_DataCaptureSystem->addSection("HazardSings", "hazard_signs");
}

void N10::onShutdown()
{
    m_WheelsRPMFeedbackSubscriber.reset();
    m_WheelsAngleFeedbackSubscriber.reset();
    m_GripperAngleFeedbackSubscriber.reset();
    m_GripperDistanceSensorSubscriber.reset();

    m_VoltagePowerManagementSubscriber.reset();
    m_VoltageAdapterSubscriber.reset();
    m_WheelsEnabledSubscriber.reset();


    m_WheelsRPMPublisher.reset();
    m_WheelsAnglePublisher.reset();
    m_GripperAnglePublisher.reset();

    m_TurtleTwistPublisher.reset();
    m_EnableMotorClient.reset();

    m_ImageSystemBackendGST.reset();
    m_DataCaptureSystem.reset();

    m_ImageSystem.reset();
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

void N10::WheelsAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    if (msg->data.size() == m_Wheels.size()) for (int i = 0; i < m_Wheels.size(); i++)
        m_Wheels[i].last_angle = msg->data[i];
}

void N10::GripperAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(m_GripperMutex);

    if (msg->data.size() == 4) for (int i = 0; i < 4; i++)
        m_Gripper.feedback_angles[i] = msg->data[i];
}

void N10::GripperDistanceSensorCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_GripperDistanceSensorDistance.store(msg->data);
}

void N10::VoltagePowerManagementCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_VoltagePowerManagement.store(msg->data);
}

void N10::VoltageAdatpterCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_VoltageAdapter.store(msg->data);
}

void N10::WheelsEnabledCallback(const std_msgs::msg::ByteMultiArray::SharedPtr msg)
{

}

void N10::EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response) 
{
    RCLCPP_INFO(node->get_logger(), "Response: success=%s, message='%s'",
                response.get()->success ? "true" : "false",
                response.get()->message.c_str());

    m_EnableMotorClientWaiting = false;
}