#include <guiniverse/image.hpp>

void Image::SetupMat(cv::Mat &mat)
{
    int cv_type;
    if (Encoding == "mono8")
    {
        cv_type = CV_8UC1;
    }
    else if (Encoding == "bgr8")
    {
        cv_type = CV_8UC3;
    }
    else if (Encoding == "rgba8")
    {
        cv_type = CV_8UC4;
    }
    else
    {
        std::cerr << "Unsupported Encoding: " << Encoding << std::endl;
        return;
    }

    mat = cv::Mat(Height, Width, cv_type, Data.data(), Step);
}
