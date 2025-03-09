#include <guiniverse/NoName/NoName.hpp>

NoName::NoName() : RobotController("NoName", "noname")
{
    m_TwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    m_TurtleTwistPublisher = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

    m_SetModeClient = node->create_client<edu_robot::srv::SetMode>("set_mode");

    m_ImageSystem = std::make_shared<ImageSystem>(node);

    m_ImageSystem->addTopic("front/color");
    m_ImageSystem->addTopic("rear/color");

    m_DataCaptureSystem = std::make_shared<DataCaptureSystem>(node);

    m_DataCaptureSystem->addSection("Barcodes", "barcode");
}

NoName::~NoName()
{
}

void NoName::SetModeClientCallback(rclcpp::Client<edu_robot::srv::SetMode>::SharedFuture response)
{
    RCLCPP_INFO(node->get_logger(), "SetMode service responded");

    m_SetModeClientWaiting = false;
}