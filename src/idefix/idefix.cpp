#include <guiniverse/idefix/idefix.hpp>

Idefix::Idefix()
    : RobotController("Idefix", "idefix")
{
}

void Idefix::OnStartup()
{
    m_IMUSubscriber = GetNode()->create_subscription<std_msgs::msg::Float32>("imu", 10, std::bind(&Idefix::IMUCallback, this, std::placeholders::_1));

    m_TwistPublisher = GetNode()->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    m_ImageSystem = std::make_shared<ImageSystem>(GetNode());

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->AddSink(6000);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(GetNode());
    m_DataCaptureSystem->AddSection("QRCodes", "qrcode");
}

void Idefix::OnShutdown()
{
    m_TwistPublisher.reset();

    m_DataCaptureSystem.reset();

    m_DataCaptureSystem.reset();

    m_ImageSystemBackendGST.reset();
    m_ImageSystem.reset();
}

void Idefix::IMUCallback(const std_msgs::msg::Float32::UniquePtr msg)
{
    m_IMUAngle.store(msg->data);
}
