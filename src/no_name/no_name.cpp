#include <guiniverse/no_name/no_name.hpp>

NoName::NoName()
    : RobotController("NoName", "noname")
{
    m_Wheels.reserve(4);

    AddWheel(0.113f, 0.075f, 0.05f, false);  // 0
    AddWheel(0.113f, -0.075f, 0.05f, true);  // 1
    AddWheel(-0.113, 0.075f, 0.05f, false);  // 2
    AddWheel(-0.113f, -0.075f, 0.05f, true); // 3
}

NoName::~NoName()
{
}

void NoName::OnStartup()
{
    for (int i = 0; i < m_Wheels.size(); i++)
    {
        m_Wheels[i].TargetRPM = 0.f;
        m_Wheels[i].LastRPM = 0.f;
    }

    m_RPMPublisher = m_Node->create_publisher<std_msgs::msg::Float32MultiArray>("set_motor_rpm", 10);
    m_TurtleTwistPublisher = m_Node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_SetModeClient = m_Node->create_client<edu_robot::srv::SetMode>("set_mode");

    m_ImageSystem = std::make_shared<ImageSystem>(m_Node);

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->AddSink(7000);
    m_ImageSystemBackendGST->AddSink(7001);
    m_ImageSystemBackendGST->AddSink(7002);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(m_Node);
    m_DataCaptureSystem->AddSection("QRCodes", "qrcode");
}

void NoName::OnShutdown()
{
    m_RPMPublisher.reset();
    m_TurtleTwistPublisher.reset();

    m_SetModeClient.reset();

    m_DataCaptureSystem.reset();

    m_ImageSystemBackendGST.reset();
    m_ImageSystem.reset();
}

void NoName::AddWheel(float x, float y, float radius, bool invert)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    ExplorerWheel wheel = { x, y, radius, invert, 0.f, 0.f };

    m_Wheels.push_back(wheel);
}

void NoName::SetModeClientCallback(rclcpp::Client<edu_robot::srv::SetMode>::SharedFuture response)
{
    RCLCPP_INFO(m_Node->get_logger(), "SetMode service responded");

    m_SetModeClientWaiting = false;
}
