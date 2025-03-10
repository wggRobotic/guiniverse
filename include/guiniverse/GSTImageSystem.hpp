#pragma once

#include <mutex>
#include <vector>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

struct GSTImageSystemImageProcessor
{
    short port;
    
    GstElement *pipeline;
    GstElement *appsink;
    GstAppSink *sink;

    GstClockTime last_frame_timestamp;

    bool holds_image = false;

    unsigned int texture;
    unsigned int texture_width = 0;
    unsigned int texture_height = 0;

    bool init = false;

    bool flip_vertically = false;
    bool flip_horizontally = false;
};

class GSTImageSystem
{

public:

    GSTImageSystem();

    void addSink(short port);

    void ImGuiPanels();

    void onGuiShutdown();

private:

    std::mutex m_ImageProcessorMutex;
    std::vector<GSTImageSystemImageProcessor> m_ImageProcessors;
};