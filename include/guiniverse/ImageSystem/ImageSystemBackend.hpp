#pragma once

#include <GL/gl.h>
#include <string>

class ImageSystemBackend
{
public:

    ImageSystemBackend() {};

    virtual ~ImageSystemBackend() = default;

    virtual int getProcessorCount() = 0;
    virtual std::string ImGuiPanelName(int index) = 0;

    virtual void onFrame() = 0;
    virtual bool onFramegetImage(int index, int* image_layout, int* width, int* height, unsigned char** data) = 0;
    virtual void onFramegotImage(int index) = 0;
};