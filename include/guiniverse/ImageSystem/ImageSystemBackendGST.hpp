#pragma once

#include <guiniverse/ImageSystem/ImageSystemBackend.hpp>

#include <vector>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

struct ImageSystemBackendGSTProcessor
{
    short port;
    
    GstElement *pipeline;
    GstElement *appsink;
    GstAppSink *sink;

    GstClockTime last_frame_timestamp;

    //Frame
    GstSample* sample;
    GstBuffer* buffer;
    GstMapInfo map;

};


class ImageSystemBackendGST : public ImageSystemBackend
{
public:
    ImageSystemBackendGST();

    ~ImageSystemBackendGST();

    int getProcessorCount() override;
    std::string ImGuiPanelName(int index) override;
    
    void onFrame() override;
    bool onFramegetImage(int index, int* image_layout, int* width, int* height, unsigned char** data) override;
    void onFramegotImage(int index) override;

    void addSink(short port);

private:

    std::vector<ImageSystemBackendGSTProcessor> m_Processors;
};