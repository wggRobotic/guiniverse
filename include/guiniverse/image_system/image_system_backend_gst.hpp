#pragma once

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <guiniverse/image_system/image_system.hpp>
#include <vector>

struct ImageSystemBackendGSTProcessor final
{
    int Index;
    short Port;

    GstElement* Pipeline;
    GstElement* AppSink;
    GstAppSink* Sink;

    GstClockTime LastFrameTimestamp;
};

class ImageSystemBackendGST final : public ImageSystemBackend
{
public:
    ImageSystemBackendGST(std::shared_ptr<ImageSystem> image_system);

    ~ImageSystemBackendGST();

    void OnFrame();

    void AddSink(short port);

private:
    std::vector<ImageSystemBackendGSTProcessor> m_Processors;
};
