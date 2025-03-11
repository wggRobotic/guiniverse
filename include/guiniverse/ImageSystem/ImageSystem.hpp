#pragma once

#include <vector>
#include <mutex>
#include <memory>

#include <GL/gl.h>

struct ImageSystemImageProcessor
{
    std::unique_ptr<std::mutex> mutex = std::make_unique<std::mutex>();

    std::string imgui_panel_name = "none";

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

    void onGuiStartup();
    void onGuiFrame();
    void onGuiShutdown();

    int addImageProcessor(const std::string& imgui_panel_name);
    void ImageCallback(int index, int image_layout, int width, int height, unsigned char* data);

private:

    std::vector<ImageSystemImageProcessor> m_ImageProcessors;
};

class ImageSystemBackend
{

public:
    ImageSystemBackend(std::shared_ptr<ImageSystem> image_system) : m_ImageSystem(image_system) {};

protected:
    std::shared_ptr<ImageSystem> m_ImageSystem;

};