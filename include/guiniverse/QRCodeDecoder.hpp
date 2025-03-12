#pragma once

#include <atomic>
#include <GL/gl.h>
#include <thread>
#include <mutex>
#include <vector>
#include <quirc.h>

class QRCodeDecoder
{
public:

    QRCodeDecoder() {m_QuircInstance = quirc_new();};
    ~QRCodeDecoder() {quirc_destroy(m_QuircInstance);};

    void start();
    void stop();

    bool isReady();

    void startDecoding(int width, int height, unsigned char* data, int format);
    void getLastResult(std::vector<std::string>& result);

private:

    void WorkerThreadFunction();

    struct quirc* m_QuircInstance;

    std::thread m_WorkerThread;

    std::atomic<bool> m_Ready{false};
    std::atomic<bool> m_isRunning{false};
    std::atomic<bool> m_shouldStop{false};

    std::mutex m_ResultsMutex;
    std::vector<std::string> m_Results;

    std::atomic<bool> m_shouldDecode{false};
    struct
    {
        std::vector<unsigned char> data;
        int width;
        int height;
        int format;
    } m_WorkData;

};