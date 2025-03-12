#pragma once

#include <vector>
#include <mutex>
#include <memory>

#include <GL/gl.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <guiniverse/QRCodeDecoder.hpp>

enum ImageSystemExtra
{
    ImageSystemExtra_None = 0,
    ImageSystemExtra_QRCodeDecoder = 1,
};

struct ImageSystemImageProcessor
{
    std::unique_ptr<std::mutex> mutex = std::make_unique<std::mutex>();

    std::string imgui_panel_name = "none";
    int extras = 0;

    struct
    {
        unsigned int gl_texture;
        unsigned int width = 0;
        unsigned int height = 0;
    } texture;

    struct
    {
        bool holds = false;
        bool dirty = false;
        int width = 0;
        int height = 0;
        int image_layout;
        std::vector<unsigned char> data;
    } image;    

    bool flip_vertically = false;
    bool flip_horizontally = false;
};

class ImageSystem
{

public:

    ImageSystem(rclcpp::Node::SharedPtr node);

    void onGuiStartup();
    void onGuiFrame();
    void onGuiShutdown();

    int addImageProcessor(int extras, const std::string& imgui_panel_name);
    void ImageCallback(int index, int image_layout, int width, int height, unsigned char* data);

private:
    std::vector<ImageSystemImageProcessor> m_ImageProcessors;
    QRCodeDecoder m_QRCodeDecoder;

    rclcpp::Node::SharedPtr m_Node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_QRCodePublisher;
    std_msgs::msg::String m_QRCodeMessae;
};

class ImageSystemBackend
{

public:
    ImageSystemBackend(std::shared_ptr<ImageSystem> image_system) : m_ImageSystem(image_system) {};

protected:
    std::shared_ptr<ImageSystem> m_ImageSystem;

};