#include <GL/gl.h>
#include <guiniverse/image_system/image_system.hpp>
#include <imgui.h>

#define GLCALL(call)                                                                              \
    do                                                                                            \
    {                                                                                             \
        call;                                                                                     \
        GLenum err = glGetError();                                                                \
        if (err != GL_NO_ERROR)                                                                   \
        {                                                                                         \
            fprintf(stderr, "OpenGL error in %s at %s:%d: %d\n", #call, __FILE__, __LINE__, err); \
        }                                                                                         \
    } while (0)

void ImageSystemImage::Clear()
{
    Image.release();
}

void ImageSystemImage::CreateTexture()
{
    GLCALL(glGenTextures(1, &TextureObject));
    GLCALL(glBindTexture(GL_TEXTURE_2D, TextureObject));

    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GLCALL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));

    GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
}

void ImageSystemImage::DestroyTexture()
{
    GLCALL(glDeleteTextures(1, &TextureObject));
    TextureWidth = 0;
    TextureHeight = 0;
}

void ImageSystemImage::ImGuiImage(bool flip_vertically, bool flip_horizontally)
{
    bool empty;

    {
        std::lock_guard<std::mutex> lock(ImageMutex);

        empty = Image.empty();

        if (!empty)
        {
            GLCALL(glBindTexture(GL_TEXTURE_2D, TextureObject));
            if (Image.cols != TextureWidth || Image.rows != TextureHeight)
            {
                TextureWidth = Image.cols;
                TextureHeight = Image.rows;
                GLCALL(glTexImage2D(
                    GL_TEXTURE_2D,
                    0,
                    GL_RGB8,
                    TextureWidth,
                    TextureHeight,
                    0,
                    GL_RGB,
                    GL_UNSIGNED_BYTE,
                    Image.data));
            }
            else
            {
                GLCALL(glTexSubImage2D(
                    GL_TEXTURE_2D,
                    0,
                    0,
                    0,
                    TextureWidth,
                    TextureHeight,
                    GL_RGB,
                    GL_UNSIGNED_BYTE,
                    Image.data));
            }
            GLCALL(glBindTexture(GL_TEXTURE_2D, 0));
        }
    }

    if (!empty)
    {
        ImVec2 widget_size = ImGui::GetContentRegionAvail();

        float image_aspect = (float) TextureWidth / (float) TextureHeight;
        float widget_aspect = widget_size.x / widget_size.y;

        ImVec2 display_size;

        if (widget_aspect > image_aspect)
            display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
        else
            display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);

        ImGui::Image((ImTextureID) TextureObject, display_size, ImVec2(flip_horizontally ? 1.f : 0.f, flip_vertically ? 1.f : 0.f), ImVec2(flip_horizontally ? 0.f : 1.f, flip_vertically ? 0.f : 1.f));
    }
    else
    {
        ImGui::Text("No image yet");
    }
}

void ImageSystemImage::SubImageTransferOwnership(cv::Mat& mat)
{
    std::lock_guard<std::mutex> lock(ImageMutex);

    Dirty = true;
    Image = mat;
}

void ImageSystemImage::CopyImage(cv::Mat& mat)
{
    std::lock_guard<std::mutex> lock(ImageMutex);

    Image.copyTo(mat);
}

ImageSystem::ImageSystem(rclcpp::Node::SharedPtr node)
    : m_Node(node)
{
    m_ImageProcessors.reserve(3);
}

ImageSystem::~ImageSystem()
{
    for (size_t i = 0; i < m_ImageProcessors.size(); i++)
    {
        m_ImageProcessors[i]->Addons.ThreadShouldClose.store(true);
        if (m_ImageProcessors[i]->Addons.Thread.joinable())
            m_ImageProcessors[i]->Addons.Thread.join();

        quirc_destroy(m_ImageProcessors[i]->Addons.QRCode.QuircInstance);
    }
}

int ImageSystem::AddImageProcessor(const std::string& imgui_panel_name)
{
    int size = m_ImageProcessors.size();
    m_ImageProcessors.resize(size + 1);
    m_ImageProcessors[size] = std::make_shared<ImageSystemImageProcessor>();

    m_ImageProcessors[size]->ImGuiPanelName = imgui_panel_name;

    m_ImageProcessors[size]->Addons.QRCode.QuircInstance = quirc_new();
    m_ImageProcessors[size]->Addons.QRCode.Publisher = m_Node->create_publisher<std_msgs::msg::String>("qrcode", 10);

    m_ImageProcessors[size]->Addons.Thread = std::thread(&ImageSystem::AddOnThreadFunction, this, size);

    return size;
}

void ImageSystem::AddOnThreadFunction(int index)
{
    while (!m_ImageProcessors[index]->Addons.ThreadShouldClose.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        cv::Mat image;
        m_ImageProcessors[index]->Image.CopyImage(image);

        // qrcode
        if (m_ImageProcessors[index]->Addons.Flags.load() & ImageSystemAddOn_QRCode
            && !image.empty())
        {
            cv::Mat grayscale;
            cv::cvtColor(image, grayscale, cv::COLOR_RGB2GRAY);

            quirc_resize(
                m_ImageProcessors[index]->Addons.QRCode.QuircInstance,
                image.cols,
                image.rows);

            int width, height;
            uint8_t* qr_image = quirc_begin(
                m_ImageProcessors[index]->Addons.QRCode.QuircInstance,
                &width,
                &height);
            memcpy(qr_image, grayscale.data, width * height);
            quirc_end(m_ImageProcessors[index]->Addons.QRCode.QuircInstance);

            int count = quirc_count(m_ImageProcessors[index]->Addons.QRCode.QuircInstance);

            for (int i = 0; i < count; i++)
            {
                struct quirc_code code;
                struct quirc_data data;

                quirc_extract(m_ImageProcessors[index]->Addons.QRCode.QuircInstance, i, &code);

                if (quirc_decode(&code, &data) == QUIRC_SUCCESS)
                {
                    std_msgs::msg::String msg;
                    msg.data = std::string((char*) data.payload);
                    m_ImageProcessors[index]->Addons.QRCode.Publisher->publish(msg);
                }
            }
        }
    }
}

void ImageSystem::ImageCallback(int index, cv::Mat& image)
{
    int addon_flags = m_ImageProcessors[index]->Addons.Flags.load();

    cv::Mat old_image;
    if (addon_flags & ImageSystemAddOn_Diff)
        m_ImageProcessors[index]->Image.CopyImage(old_image);

    m_ImageProcessors[index]->Image.SubImageTransferOwnership(image);

    if (old_image.cols == image.cols && old_image.rows == image.rows && addon_flags & ImageSystemAddOn_Diff)
    {
        cv::Mat diff_image;
        cv::absdiff(image, old_image, diff_image);

        m_ImageProcessors[index]->Addons.Diff.Image.SubImageTransferOwnership(diff_image);
    }

    if (addon_flags & ImageSystemAddon_GrayScale)
    {
        cv::Mat grayscale;
        cv::Mat grayscale_rgb;

        cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
        cv::cvtColor(grayscale, grayscale_rgb, cv::COLOR_GRAY2BGR);

        m_ImageProcessors[index]->Addons.Grayscale.Image.SubImageTransferOwnership(grayscale_rgb);
    }
}

void ImageSystem::OnGuiStartup()
{
    for (size_t i = 0; i < m_ImageProcessors.size(); i++)
    {
        m_ImageProcessors[i]->Image.CreateTexture();
        m_ImageProcessors[i]->Addons.Diff.Image.CreateTexture();
        m_ImageProcessors[i]->Addons.Grayscale.Image.CreateTexture();
    }
}

void ImageSystem::OnGuiShutdown()
{
    for (size_t i = 0; i < m_ImageProcessors.size(); i++)
    {
        m_ImageProcessors[i]->Image.DestroyTexture();
        m_ImageProcessors[i]->Addons.Diff.Image.DestroyTexture();
        m_ImageProcessors[i]->Addons.Grayscale.Image.DestroyTexture();
    }
}

void ImageSystem::OnGuiFrame()
{
    for (size_t i = 0; i < m_ImageProcessors.size(); i++)
    {
        int addon_flags = m_ImageProcessors[i]->Addons.Flags.load();
        bool qrcode_addon = (addon_flags & ImageSystemAddOn_QRCode ? true : false);
        bool diff_addon = (addon_flags & ImageSystemAddOn_Diff ? true : false);
        bool grayscale_addon = (addon_flags & ImageSystemAddon_GrayScale ? true : false);

        if (ImGui::Begin(m_ImageProcessors[i]->ImGuiPanelName.c_str()))
        {
            ImGui::Checkbox("Flip vertically", &m_ImageProcessors[i]->FlipVertically);
            ImGui::Checkbox("Flip horizontally", &m_ImageProcessors[i]->FlipHorizontally);

            ImGui::Checkbox("QRCode scanning", &qrcode_addon);
            ImGui::Checkbox("Image diff", &diff_addon);
            ImGui::Checkbox("Image grayscale", &grayscale_addon);

            m_ImageProcessors[i]->Image.ImGuiImage(
                m_ImageProcessors[i]->FlipVertically,
                m_ImageProcessors[i]->FlipHorizontally);
        }
        ImGui::End();

        if (addon_flags & ImageSystemAddOn_Diff)
        {
            if (ImGui::Begin((m_ImageProcessors[i]->ImGuiPanelName + " - diff").c_str()))
            {
                m_ImageProcessors[i]->Addons.Diff.Image.ImGuiImage(
                    m_ImageProcessors[i]->FlipVertically,
                    m_ImageProcessors[i]->FlipHorizontally);
            }
            ImGui::End();
        }

        if (addon_flags & ImageSystemAddon_GrayScale)
        {
            if (ImGui::Begin(
                    (m_ImageProcessors[i]->ImGuiPanelName + " - grayscale").c_str()))
            {
                m_ImageProcessors[i]->Addons.Grayscale.Image.ImGuiImage(
                    m_ImageProcessors[i]->FlipVertically,
                    m_ImageProcessors[i]->FlipHorizontally);
            }
            ImGui::End();
        }

        m_ImageProcessors[i]->Addons.Flags.store((qrcode_addon ? ImageSystemAddOn_QRCode : 0) | (diff_addon ? ImageSystemAddOn_Diff : 0) | (grayscale_addon ? ImageSystemAddon_GrayScale : 0));
    }
}
