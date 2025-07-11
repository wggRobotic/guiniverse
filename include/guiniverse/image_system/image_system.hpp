#pragma once

#include <GL/gl.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <quirc.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <vector>

struct ImageSystemImage final
{
    std::mutex ImageMutex;
    cv::Mat Image;

    GLuint TextureObject;
    int TextureWidth = 0;
    int TextureHeight = 0;

    bool Dirty = false;

    void Clear();

    void CreateTexture();
    void DestroyTexture();
    void ImGuiImage(bool flip_vertically, bool flip_horizontally);

    void SubImageTransferOwnership(cv::Mat& mat);
    void CopyImage(cv::Mat& mat);
};

enum ImageSystemAddOn
{
    ImageSystemAddOn_None = 0,
    ImageSystemAddOn_QRCode = 1,
    ImageSystemAddOn_Diff = 2,
    ImageSystemAddon_GrayScale = 4,
};

struct ImageSystemImageProcessor final
{
    struct
    {
        std::atomic<int> Flags{ ImageSystemAddOn_None };
        std::thread Thread;
        std::atomic<bool> ThreadShouldClose{ false };

        struct
        {
            struct quirc* QuircInstance;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publisher;
        } QRCode;

        struct
        {
            ImageSystemImage Image;
        } Diff;

        struct
        {
            ImageSystemImage Image;
        } Grayscale;

    } Addons;

    std::string ImGuiPanelName = "none";

    bool FlipVertically = false;
    bool FlipHorizontally = false;

    ImageSystemImage Image;
};

class ImageSystem final
{
public:
    ImageSystem(rclcpp::Node::SharedPtr node);
    ~ImageSystem();

    void OnGuiStartup();
    void OnGuiFrame();
    void OnGuiShutdown();

    int AddImageProcessor(const std::string& imgui_panel_name);
    void ImageCallback(int index, cv::Mat& image);

private:
    void AddOnThreadFunction(int index);

    std::vector<std::shared_ptr<ImageSystemImageProcessor>> m_ImageProcessors;
    rclcpp::Node::SharedPtr m_Node;
};

class ImageSystemBackend
{
public:
    ImageSystemBackend(std::shared_ptr<ImageSystem> image_system)
        : m_ImageSystem(image_system) { };

protected:
    std::shared_ptr<ImageSystem> m_ImageSystem;
};
