#pragma once

#include <guiniverse/ImageSystem/ImageSystem.hpp>

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

struct ImageSystemBackendROSProcessor
{
    int index;

    std::string topic_name;
    image_transport::Subscriber subscriber;
};


class ImageSystemBackendROS : public ImageSystemBackend
{
public:
    ImageSystemBackendROS(std::shared_ptr<ImageSystem> image_system, std::shared_ptr<rclcpp::Node> node);

    ~ImageSystemBackendROS();

    void addSubscriber(const std::string& topic_name);

private:
    std::vector<ImageSystemBackendROSProcessor> m_Processors;

    std::shared_ptr<rclcpp::Node> m_Node;
    std::shared_ptr<image_transport::ImageTransport> m_ImageTransport;
};