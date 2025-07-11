#pragma once

#include <guiniverse/image_system/image_system.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

struct ImageSystemBackendROSProcessor final
{
    int Index;

    std::string TopicName;
    image_transport::Subscriber Subscriber;
};

class ImageSystemBackendROS final : public ImageSystemBackend
{
public:
    ImageSystemBackendROS(std::shared_ptr<ImageSystem> image_system, std::shared_ptr<rclcpp::Node> node);

    ~ImageSystemBackendROS();

    void AddSubscriber(const std::string& topic_name);

private:
    std::vector<ImageSystemBackendROSProcessor> m_Processors;

    std::shared_ptr<rclcpp::Node> m_Node;
    std::shared_ptr<image_transport::ImageTransport> m_ImageTransport;
};
