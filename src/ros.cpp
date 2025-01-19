#include <guiniverse/node.hpp>
#include <guiniverse/ros.hpp>
#include <image_transport/image_transport.hpp>

void ros_thread()
{
    auto node = std::make_shared<GuiniverseNode>();
    image_transport::ImageTransport image_transport(node);
    node->SetupWithImageTransport(image_transport);
    node->run();
}
