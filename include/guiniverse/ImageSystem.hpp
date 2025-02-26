#pragma once

#include <opencv2/opencv.hpp>

#include <mutex>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>


struct Image
{
    void SetupMat(cv::Mat &mat);

    uint32_t Width;
    uint32_t Height;
    uint32_t Step;
    std::string Encoding;
    std::vector<uint8_t> Data;
};

using ImageData = std::pair<Image, bool>;

struct ImageSystemImageProcessor
{
    std::string topic_name;

    image_transport::Subscriber subscriber;
    
    Image image;
    bool holds_image = false;

    unsigned int texture;
    unsigned int texture_width = 0;
    unsigned int texture_height = 0;

    bool dirty = false;
};

class ImageSystem : public image_transport::ImageTransport
{

public:

    ImageSystem(std::shared_ptr<rclcpp::Node> Node);

    ~ImageSystem();

    void addTopic(const std::string& TopicName);

    void setImage(size_t index, const std::vector<uint8_t> &data, uint32_t width, uint32_t height, uint32_t step, const std::string &encoding);

    void ImGuiPanels();

private:

    std::shared_ptr<rclcpp::Node> m_Node;

    std::mutex m_ImageProcessorMutex;
    std::vector<ImageSystemImageProcessor> m_ImageProcessors;
};