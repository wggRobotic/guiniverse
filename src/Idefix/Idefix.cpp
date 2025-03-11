#include <guiniverse/Idefix/Idefix.hpp>

Idefix::Idefix() : RobotController("Idefix", "idefix")
{
    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_TurtleTwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);

    m_DataCaptureSystem->addSection("Barcodes", "barcode");
}

Idefix::~Idefix()
{
}