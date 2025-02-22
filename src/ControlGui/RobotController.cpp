#include <guiniverse/ControlGui/RobotController.hpp>

RobotController::RobotController(const std::string& RobotName, const std::string& RosRobotName) 
: rclcpp::Node(RosRobotName + "_controller_node"), m_RobotName(RobotName), m_RosRobotName(RosRobotName) 
{


    
}