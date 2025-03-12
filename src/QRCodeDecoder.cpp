#include <guiniverse/QRCodeDecoder.hpp>

#include <string.h>

int _getPixelSize(int format)
{
    switch (format)
    {
    case GL_RGB:
        return 3;
    case GL_BGR:
        return 3;
    case GL_R:
        return 1;
    case GL_RGBA:
        return 4;

    default:
        return 1;
    }
}

void QRCodeDecoder::start()
{
    if (m_isRunning.load()) return;
    m_WorkerThread = std::thread(&QRCodeDecoder::WorkerThreadFunction, this);
}

void QRCodeDecoder::stop()
{
    if (!m_isRunning.load()) return;
    m_shouldStop.store(true);
    if (m_WorkerThread.joinable()) m_WorkerThread.join();
}

bool QRCodeDecoder::isReady()
{
    return m_Ready.load();
}

void QRCodeDecoder::startDecoding(int width, int height, unsigned char* data, int format)
{
    if (!m_Ready.load()) return;

    m_WorkData.format = format;
    m_WorkData.width = width;
    m_WorkData.height = height;
    m_WorkData.data.assign(data, data + _getPixelSize(format) * width * height);

    m_shouldDecode.store(true);
    m_Ready.store(false);
}

void QRCodeDecoder::getLastResult(std::vector<std::string>& result)
{
    std::lock_guard<std::mutex> lock(m_ResultsMutex);
    result = m_Results;
}

void QRCodeDecoder::WorkerThreadFunction()
{
    m_isRunning.store(true);
    m_Ready.store(true);

    while (!m_shouldStop.load())
    {
        if (!m_shouldDecode.load()) 
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        int width = m_WorkData.width, height = m_WorkData.height, format = m_WorkData.format;

        std::vector<unsigned char> gray_scale(width * height);

        switch (format)
        {
        case GL_RGB: for (int i = 0, offset = 0; i < width * height; i++, offset+=3) {
            gray_scale[i] = (unsigned char)(
                0.299f * (float)m_WorkData.data[offset + 0] + 
                0.587f * (float)m_WorkData.data[offset + 1] + 
                0.114f * (float)m_WorkData.data[offset + 2]
            );
        } break;

        case GL_BGR: for (int i = 0, offset = 0; i < width * height; i++, offset+=3) {
            gray_scale[i] = (unsigned char)(
                0.299f * (float)m_WorkData.data[offset + 2] + 
                0.587f * (float)m_WorkData.data[offset + 1] + 
                0.114f * (float)m_WorkData.data[offset + 0]
            );
        } break;

        case GL_R: for (int i = 0, offset = 0; i < width * height; i++, offset+=1) {
            gray_scale[i] = (unsigned char)(
                0.299f * (float)m_WorkData.data[offset + 0]
            );
        } break;

        case GL_RGBA: for (int i = 0, offset = 0; i < width * height; i++, offset+=4) {
            gray_scale[i] = (unsigned char)(
                0.299f * (float)m_WorkData.data[offset + 0] + 
                0.587f * (float)m_WorkData.data[offset + 1] + 
                0.114f * (float)m_WorkData.data[offset + 2]
            );
        } break;

        default: for (int i = 0, offset = 0; i < width * height; i++, offset+=1) {
            gray_scale[i] = (unsigned char)(
                0.299f * (float)m_WorkData.data[offset + 0]
            );
        } break;

        }

        quirc_resize(m_QuircInstance, width, height);

        uint8_t *qr_image = quirc_begin(m_QuircInstance, &width, &height);
        memcpy(qr_image, gray_scale.data(), width * height);
        quirc_end(m_QuircInstance);

        int count = quirc_count(m_QuircInstance);

        std::lock_guard<std::mutex> lock(m_ResultsMutex);
        m_Results.clear();
        m_Results.reserve(count);

        for (int i = 0; i < count; i++) {
            struct quirc_code code;
            struct quirc_data data;

            quirc_extract(m_QuircInstance, i, &code);

            if (quirc_decode(&code, &data) == QUIRC_SUCCESS) {
                m_Results.push_back(std::string((char*)data.payload));
            } 
        }

        m_Ready.store(true);
    }

    m_isRunning.store(false);
}