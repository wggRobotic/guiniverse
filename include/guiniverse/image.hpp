#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct Image
{
    void SetupMat(cv::Mat &mat);

    uint32_t Width;
    uint32_t Height;
    uint32_t Step;
    std::string Encoding;
    std::vector<uint8_t> Data;
};
