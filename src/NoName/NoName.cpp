#include <guiniverse/NoName/NoName.hpp>

NoName::NoName() : RobotController("NoName", "noname")
{
    m_Wheels.reserve(4);

    addWheel(  0.113f,  0.075f, 0.05f, false); //0
    addWheel(  0.113f, -0.075f, 0.05f, true ); //1
    addWheel( -0.113,   0.075f, 0.05f, false); //2
    addWheel( -0.113f, -0.075f, 0.05f, true ); //3
}

NoName::~NoName()
{
}

void NoName::onStartup()
{
    for (int i = 0; i < m_Wheels.size(); i++)
    {
        m_Wheels[i].target_rpm = 0.f;
        m_Wheels[i].last_rpm = 0.f;
    }

    m_RPMPublisher = node->create_publisher<std_msgs::msg::Float32MultiArray>("set_motor_rpm", 10);
    m_TurtleTwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_SetModeClient = node->create_client<edu_robot::srv::SetMode>("set_mode");

    m_ImageSystem = std::make_shared<ImageSystem>(node);

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->addSink(7000);
    m_ImageSystemBackendGST->addSink(7001);
    m_ImageSystemBackendGST->addSink(7002);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);
    m_DataCaptureSystem->addSection("QRCodes", "qrcode");
}

void NoName::onShutdown()
{
    m_RPMPublisher.reset();
    m_TurtleTwistPublisher.reset();

    m_SetModeClient.reset();

    m_DataCaptureSystem.reset();

    m_ImageSystemBackendGST.reset();
    m_ImageSystem.reset();
}


void NoName::addWheel(float x, float y, float radius, bool invert)
{
    std::lock_guard<std::mutex> lock(m_WheelsMutex);

    ExplorerWheel wheel = {x, y, radius, invert, 0.f, 0.f};

    m_Wheels.push_back(wheel);
}

void NoName::SetModeClientCallback(rclcpp::Client<edu_robot::srv::SetMode>::SharedFuture response)
{
    RCLCPP_INFO(node->get_logger(), "SetMode service responded");

    m_SetModeClientWaiting = false;
}