#pragma once

#include <guiniverse/ImageSystem/ImageSystemBackend.hpp>

#include <vector>
#include <mutex>
#include <memory>


static int ImageSystemImageLayoutPixelSize(int image_layout)
{
    switch (image_layout)
    {
    case GL_RGB:
        return 3;
    case GL_BGR:
        return 3;
    
    default:
        return 4;
    }
}

struct ImageSystemImageProcessor
{
    std::unique_ptr<std::mutex> mutex;

    int backend_index;
    int backend_processor_index;

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

    void addImageSystemBackend(ImageSystemBackend* backend);

    void onFrame();

    void onGuiStartup();

    void onGuiFrame();

    void onGuiShutdown();

private:

    std::vector<ImageSystemImageProcessor> m_ImageProcessors;
    std::vector<ImageSystemBackend*> m_Backends;
};