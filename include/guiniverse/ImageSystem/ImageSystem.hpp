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
    ImageSystemAddOn_Diff = 2,
};

struct ImageSystemImageProcessor
{
    //image
    struct
    {
        std::mutex mutex;

        bool holds = false;
        bool dirty = false;
        int width = 0;
        int height = 0;
        bool is_bgr;
        std::vector<unsigned char> data;
    } image;   

    struct
    {
        int flags = 0;
        std::thread thread;
        std::atomic<bool> thread_should_close{false};

        struct
        {
            struct quirc* quirc_instance;
            std::vector<unsigned char> gray_scale;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        } qrcode;

        struct
        {
            struct
            {
                std::mutex mutex;

                bool holds = false;
                bool dirty = false;
                int width = 0;
                int height = 0;
                std::vector<unsigned char> data;
            } image;   
        
            struct
            {
                std::atomic<float> diff_intensity{4.f};

                unsigned int gl_texture;
                unsigned int width = 0;
                unsigned int height = 0;
            } imgui;
        } diff;

    } addons;

    struct
    {
        std::string panel_name = "none";

        bool flip_vertically = false;
        bool flip_horizontally = false;

        unsigned int gl_texture;
        unsigned int width = 0;
        unsigned int height = 0;
    } imgui;

    
};

class ImageSystem
{

public:

    ImageSystem(rclcpp::Node::SharedPtr node);
    ~ImageSystem();

    void onGuiStartup();
    void onGuiFrame();
    void onGuiShutdown();

    int addImageProcessor(int addon_flags, const std::string& imgui_panel_name);
    void ImageCallback(int index, bool is_bgr, int width, int height, unsigned char* data);

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