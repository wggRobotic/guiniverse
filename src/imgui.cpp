#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

#include <guiniverse/shared_data.hpp>

int selectedImageIndex = 0;
GLuint currentTexture = 0;

static void set_style()
{
    ImVec4 *colors = ImGui::GetStyle().Colors;
}

GLuint convertToTexture(const cv::Mat &image)
{
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // OpenCV-Bilddaten verwenden
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image.data);

    // Texturparameter setzen
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
    return textureID;
}

static void on_key(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
    {
        running = false;
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void updateTexture()
{
    // Lösche vorherige Textur, falls vorhanden
    if (currentTexture != 0)
    {
        glDeleteTextures(1, &currentTexture);
        currentTexture = 0;
    }

    // Lock shared_image_data und lade neue Textur
    {
        std::lock_guard<std::mutex> lock(image_mutex);
        if (selectedImageIndex < shared_image_data.size())
        {
            currentTexture = convertToTexture(shared_image_data[selectedImageIndex]);
        }
    }
}

void imgui_thread()
{
    if (!glfwInit())
        return;

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow *window = glfwCreateWindow(800, 600, "GUINIVERSE", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return;
    }

    glfwSetKeyCallback(window, on_key);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glEnable(GL_MULTISAMPLE);
    glewInit();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    auto &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    float clear_color[]{0.00f, 0.00f, 0.00f, 0.35f};
    bool show_styles = false;

    while (!glfwWindowShouldClose(window) && running)
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        updateTexture();

        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport()->ID, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        std::string display_data;
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex);
            display_data = shared_data;
        }

        if (ImGui::Begin("Home"))
        {
            ImGui::Text("Shared Data: %s", display_data.c_str());
            ImGui::ColorEdit4("Clear Color", clear_color);
            ImGui::Checkbox("Show Styles", &show_styles);
        }
        ImGui::End();

        if (ImGui::Begin("Image Viewer"))
        {
            if (ImGui::BeginCombo("Select Image", ("Image " + std::to_string(selectedImageIndex)).c_str()))
            {
                std::lock_guard<std::mutex> lock(image_mutex);

                for (size_t i = 0; i < shared_image_data.size(); i++)
                {
                    const bool isSelected = (selectedImageIndex == i);
                    if (ImGui::Selectable(("Image " + std::to_string(i)).c_str(), isSelected))
                    {
                        selectedImageIndex = i;
                    }
                    if (isSelected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }

            if (currentTexture != 0)
            {
                std::lock_guard<std::mutex> lock(image_mutex);
                if (selectedImageIndex < shared_image_data.size())
                {
                    auto &image = shared_image_data[selectedImageIndex];
                    ImVec2 widget_size = ImGui::GetContentRegionAvail();

                    float image_aspect = static_cast<float>(image.cols) / image.rows;
                    float widget_aspect = widget_size.x / widget_size.y;

                    ImVec2 display_size;
                    if (widget_aspect > image_aspect)
                    {
                        display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                    }
                    else
                    {
                        display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                    }

                    ImGui::Image(reinterpret_cast<ImTextureID>(static_cast<void *>(reinterpret_cast<void *>(currentTexture))),
                                 display_size);
                }
            }
        }
        ImGui::End();

        if (ImGui::Begin("Control"))
        {
        }
        ImGui::End();

        if (ImGui::Begin("Sensor Data"))
        {
        }
        ImGui::End();

        if (ImGui::Begin("Visualization"))
        {
        }
        ImGui::End();

        if (show_styles)
        {
            if (ImGui::Begin("Styles", &show_styles))
                ImGui::ShowStyleEditor();
            ImGui::End();
        }

        ImGui::Render();

        glClearColor(clear_color[0], clear_color[1], clear_color[2], clear_color[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(window);
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    running = false;
}
