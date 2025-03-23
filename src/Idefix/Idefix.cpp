#include <guiniverse/Idefix/Idefix.hpp>

Idefix::Idefix() : RobotController("Idefix", "idefix")
{   
}

Idefix::~Idefix()
{
}

void Idefix::onStartup()
{
    m_IMUSubscriber = node->create_subscription<std_msgs::msg::Float32>("imu", 10, std::bind(&Idefix::IMUCallback, this, std::placeholders::_1));

    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_TurtleTwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_ImageSystem = std::make_shared<ImageSystem>(node);

    m_ImageSystemBackendGST = std::make_shared<ImageSystemBackendGST>(m_ImageSystem);
    m_ImageSystemBackendGST->addSink(6000, ImageSystemAddOn_QRCode);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);
    m_DataCaptureSystem->addSection("Barcodes", "barcode");
}

void Idefix::onShutdown()
{
    m_TwistPublisher.reset();
    m_TurtleTwistPublisher.reset();

    m_DataCaptureSystem.reset();

    m_DataCaptureSystem.reset();
    
    m_ImageSystemBackendGST.reset();
    m_DataCaptureSystem.reset();
}

void Idefix::IMUCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    m_IMUAngle.store(msg->data);
}