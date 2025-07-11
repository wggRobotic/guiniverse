#include <cstddef>
#include <guiniverse/n10/n10.hpp>

N10::N10()
    : RobotController("N10", "n10")
{
    m_GripperAngleMessage.data.resize(4);
    for (size_t i = 0; i < m_GripperAngleMessage.data.size(); i++)
        m_GripperAngleMessage.data[i] = m_Gripper.DrivePositionAngles[i];

    m_Wheels.reserve(6);

    AddWheel(0.152f, 0.105f, 0.055f, true);    // 0
    AddWheel(0.152f, -0.105f, 0.055f, false);  // 1
    AddWheel(0.000f, 0.105f, 0.052f, true);    // 2
    AddWheel(0.000f, -0.105f, 0.052f, false);  // 3
    AddWheel(-0.152f, 0.105f, 0.055f, true);   // 4
    AddWheel(-0.152f, -0.105f, 0.055f, false); // 5
}

N10::~N10()
{
}

void N10::OnStartup()
{
    for (size_t i = 0; i < m_Wheels.size(); i++)
    {
        m_Wheels[i].TargetRPM = 0.f;
        m_Wheels[i].TargetAngle = 0.f;
        m_Wheels[i].LastRPM = 0.f;
        m_Wheels[i].LastAngle = 0.f;
    }

    m_EnableMotorClientWaiting = false;

    m_WheelsRPMFeedbackSubscriber = GetNode()->create_subscription<std_msgs::msg::Float32MultiArray>("wheels/rpm/feedback", 10, std::bind(&N10::WheelsRPMFeedbackCallback, this, std::placeholders::_1));
    m_WheelsAngleFeedbackSubscriber = GetNode()->create_subscription<std_msgs::msg::Float32MultiArray>("wheels/angle/feedback", 10, std::bind(&N10::WheelsAngleFeedbackCallback, this, std::placeholders::_1));
    m_GripperAngleFeedbackSubscriber = GetNode()->create_subscription<std_msgs::msg::Float32MultiArray>("gripper/angle/feedback", 10, std::bind(&N10::GripperAngleFeedbackCallback, this, std::placeholders::_1));
    m_GripperDistanceSensorSubscriber = GetNode()->create_subscription<std_msgs::msg::Float32>("gripper/distance_sensor", 10, std::bind(&N10::GripperDistanceSensorCallback, this, std::placeholders::_1));

    m_VoltagePowerManagementSubscriber = GetNode()->create_subscription<std_msgs::msg::Float32>("voltagePwrMgmt", 10, std::bind(&N10::VoltagePowerManagementCallback, this, std::placeholders::_1));
    m_VoltageAdapterSubscriber = GetNode()->create_subscription<std_msgs::msg::Float32>("voltageAdapter", 10, std::bind(&N10::VoltageAdatpterCallback, this, std::placeholders::_1));
    m_WheelsEnabledSubscriber = GetNode()->create_subscription<std_msgs::msg::ByteMultiArray>("wheels/enabled", 10, std::bind(&N10::WheelsEnabledCallback, this, std::placeholders::_1));

    m_WheelsRPMPublisher = GetNode()->create_publisher<std_msgs::msg::Float32MultiArray>("wheels/rpm/cmd", 10);
    m_WheelsAnglePublisher = GetNode()->create_publisher<std_msgs::msg::Float32MultiArray>("wheels/angle/cmd", 10);
    m_GripperAnglePublisher = GetNode()->create_publisher<std_msgs::msg::Float32MultiArray>("gripper/angle/cmd", 10);

    m_EnableMotorClient = GetNode()->create_client<std_srvs::srv::SetBool>("whe"
                                                                           "els"
                                                                           "/en"
                                                                           "abl"
                                                                           "e");

    m_ImageSystem = std::make_shared<ImageSystem>(GetNode());

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->AddSink(5000);
    m_ImageSystemBackendGST->AddSink(5001);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(GetNode());
    m_DataCaptureSystem->AddSection("QRCodes", "qrcode");
}

void N10::OnShutdown()
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

    m_EnableMotorClient.reset();

    m_ImageSystemBackendGST.reset();
    m_ImageSystem.reset();

    m_DataCaptureSystem.reset();
}

void N10::AddWheel(float x, float y, float radius, bool invert)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    RoverWheel wheel = { x, y, radius, invert, 0.f, 0.f, 0.f, 0.f };

    m_Wheels.push_back(wheel);
}

void N10::WheelsRPMFeedbackCallback(const std_msgs::msg::Float32MultiArray::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    if (msg->data.size() == m_Wheels.size())
        for (unsigned i = 0; i < m_Wheels.size(); i++)
            m_Wheels.at(i).LastRPM = msg->data.at(i);
}

void N10::WheelsAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    if (msg->data.size() == m_Wheels.size())
        for (unsigned i = 0; i < m_Wheels.size(); i++)
            m_Wheels[i].LastAngle = msg->data[i];
}

void N10::GripperAngleFeedbackCallback(const std_msgs::msg::Float32MultiArray::UniquePtr msg)
{
    std::lock_guard<std::mutex> lock(m_GripperMutex);

    if (msg->data.size() == 4)
        for (int i = 0; i < 4; i++)
            m_Gripper.FeedbackAngles[i] = msg->data[i];
}

void N10::GripperDistanceSensorCallback(const std_msgs::msg::Float32::UniquePtr msg)
{
    m_GripperDistanceSensorDistance.store(msg->data);
}

void N10::VoltagePowerManagementCallback(const std_msgs::msg::Float32::UniquePtr msg)
{
    m_VoltagePowerManagement.store(msg->data);
}

void N10::VoltageAdapterCallback(const std_msgs::msg::Float32::UniquePtr msg)
{
    m_VoltageAdapter.store(msg->data);
}

void N10::WheelsEnabledCallback(const std_msgs::msg::ByteMultiArray::UniquePtr msg)
{
}

void N10::EnableMotorClientCallback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture response)
{
    RCLCPP_INFO(
        GetNode()->get_logger(),
        "Response: success=%s, message='%s'",
        response.get()->success ? "true" : "false",
        response.get()->message.c_str());

    m_EnableMotorClientWaiting = false;
}
