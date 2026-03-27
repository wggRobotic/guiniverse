#pragma once

#include <guiniverse/ImageSystem/ImageSystem.hpp>

#include <vector>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

struct ImageSystemBackendGSTProcessor
{
    int index;

    short port;
    
    GstElement *pipeline;
    GstElement *appsink;
    GstAppSink *sink;

    GstClockTime last_frame_timestamp;
};


class ImageSystemBackendGST : public ImageSystemBackend
{
public:
    ImageSystemBackendGST(std::shared_ptr<ImageSystem> image_system);

    ~ImageSystemBackendGST();
    
    void onFrame();
    
    void addSink(short port);

private:

    std::vector<ImageSystemBackendGSTProcessor> m_Processors;
};