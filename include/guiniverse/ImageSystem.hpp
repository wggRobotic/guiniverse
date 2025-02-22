#pragma once

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

class ImageSystem : public image_transport::ImageTransport
{

public:

    ImageSystem(std::shared_ptr<rclcpp::Node> Node);

    void ImGuiPanels();

private:

    std::shared_ptr<rclcpp::Node> m_Node;

};