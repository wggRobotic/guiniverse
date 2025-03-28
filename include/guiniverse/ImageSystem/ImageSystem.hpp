#pragma once

#include <vector>
#include <mutex>
#include <memory>
#include <thread>
#include <atomic>

#include <GL/gl.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/opencv.hpp>

#include <quirc.h>

struct ImageSystemImage
{
    std::mutex image_mutex;
    cv::Mat image;

    unsigned int gl_texture;
    unsigned int texture_width = 0;
    unsigned int texture_height = 0;

    bool dirty = false;

    void clear();

    void create_texture();
    void destroy_texture();
    void imgui_image(bool flip_vertically, bool flip_horizontally);

    void sub_image_transfer_ownership(cv::Mat& mat);
    void copy_image(cv::Mat& mat);
};

enum ImageSystemAddOn
{
    ImageSystemAddOn_None = 0,
    ImageSystemAddOn_QRCode = 1,
    ImageSystemAddOn_Diff = 2,
};

struct ImageSystemImageProcessor
{
    struct
    {
        int flags = 0;
        std::thread thread;
        std::atomic<bool> thread_should_close{false};

        struct
        {
            struct quirc* quirc_instance;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
        } qrcode;

        struct
        {
            ImageSystemImage image;
        
            std::atomic<float> diff_intensity{4.f};
        } diff;

        struct
        {
            ImageSystemImage image;
        } grayscale;

    } addons;

    std::string imgui_panel_name = "none";

    bool flip_vertically = false;
    bool flip_horizontally = false;
  
    ImageSystemImage image;
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
    void ImageCallback(int index, cv::Mat& image);

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