#pragma once

#include <vector>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>

#include <GL/gl.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <quirc.h>

enum ImageSystemAddOn
{
    ImageSystemAddOn_None = 0,
    ImageSystemAddOn_QRCode = 1,
    ImageSystemAddOn_HazardSigns = 2,
    ImageSystemAddOn_Diff = 4,
};

struct ImageSystemImageProcessor
{
    //image
    std::mutex image_mutex;

    struct
    {
        bool holds = false;
        bool dirty = false;
        int width = 0;
        int height = 0;
        int image_layout;
        std::vector<unsigned char> data;
    } image;   

    //addons
    int addons = 0;
    std::thread addon_thread;
    std::atomic<bool> thread_should_close{false};

    //qrcode addon
    struct
    {
        struct quirc* quirc_instance;
        std::vector<unsigned char> gray_scale;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    } qrcode_addon;

    //HazardSign

    //imgui
    std::string imgui_panel_name = "none";

    struct
    {
        unsigned int gl_texture;
        unsigned int width = 0;
        unsigned int height = 0;
    } texture;

    bool flip_vertically = false;
    bool flip_horizontally = false;
};

class ImageSystem
{

public:

    ImageSystem(rclcpp::Node::SharedPtr node);
    ~ImageSystem();

    void onGuiStartup();
    void onGuiFrame();
    void onGuiShutdown();

    int addImageProcessor(int addons, const std::string& imgui_panel_name);
    void ImageCallback(int index, int image_layout, int width, int height, unsigned char* data);

private:

    void AddOnThreadFunction(int index);

    std::vector<std::shared_ptr<ImageSystemImageProcessor>> m_ImageProcessors;
    rclcpp::Node::SharedPtr m_Node;
};

class ImageSystemBackend
{

public:
    ImageSystemBackend(std::shared_ptr<ImageSystem> image_system) : m_ImageSystem(image_system) {};

protected:
    std::shared_ptr<ImageSystem> m_ImageSystem;

};