#include <guiniverse/ImageSystem.hpp>

ImageSystem::ImageSystem(std::shared_ptr<rclcpp::Node> Node) : image_transport::ImageTransport(Node), m_Node(Node)
{

}

void ImageSystem::ImGuiPanels() {
    
}