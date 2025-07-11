#include <guiniverse/robot_controller.hpp>

RobotController::RobotController(const std::string& robot_name, const std::string& ros_robot_name)
    : m_RobotName(robot_name), m_RosRobotName(ros_robot_name) { };

const char* RobotController::GetRobotNameC() const
{
    return m_RobotName.c_str();
}

const std::string& RobotController::GetRobotName() const
{
    return m_RobotName;
}

const std::string& RobotController::GetRosRobotName() const
{
    return m_RosRobotName;
}

rclcpp::Node::SharedPtr RobotController::GetNode() const
{
    return m_Node;
}

void RobotController::Start()
{
    m_Node = std::make_shared<rclcpp::Node>(m_RosRobotName + "_controller_node", "/" + m_RosRobotName);
    OnStartup();
};

void RobotController::Shutdown()
{
    OnShutdown();
    m_Node.reset();
};
