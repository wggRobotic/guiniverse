#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <fstream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <guiniverse/imgui.hpp>
#include <guiniverse/shared_data.hpp>
#include <guiniverse/input.hpp>
#include <guiniverse/rover_controller.hpp>
#include <imgui.h>

static size_t selected_image_index = 0;
static std::vector<ImageData> image_data;
static std::vector<GLuint> gl_textures;

static void swap_image_data()
{
    std::lock_guard<std::mutex> lock(image_mutex);
    image_data = shared_image_data;
    gl_textures.resize(image_data.size());
}

static void set_style()
{
    auto colors = ImGui::GetStyle().Colors;
    (void)colors;
}

static void on_key(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
    {
        running = false;
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

static void init_texture(GLuint &texture, GLsizei width, GLsizei height)
{
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);

    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGB8, width, height);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_2D, 0);
}

static void update_texture(GLuint texture, const Image &image)
{
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, image.Width, image.Height, GL_BGR, GL_UNSIGNED_BYTE, image.Data.data());
    glBindTexture(GL_TEXTURE_2D, 0);
}

static void refresh_image_data()
{
    if (selected_image_index >= gl_textures.size() || selected_image_index >= image_data.size())
        return;

    auto &texture = gl_textures[selected_image_index];
    auto &[image, dirty] = image_data[selected_image_index];

    bool no_texture = !texture;
    if (no_texture)
        init_texture(texture, image.Width, image.Height);

    if (dirty || no_texture)
        update_texture(texture, image);

    dirty = false;
}

void imgui_thread()
{
    if (!glfwInit())
        return;

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    auto window = glfwCreateWindow(800, 600, "GUINIVERSE", nullptr, nullptr);
    if (!window)
    {
        glfwTerminate();
        return;
    }

    input_state input(window);
    rover_controller controller;

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

        swap_image_data();
        refresh_image_data();

        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

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
            if (ImGui::BeginCombo("Select Image", (std::string(IMAGE_TOPICS.at(selected_image_index))).c_str()))
            {

                for (size_t i = 0; i < image_data.size(); i++)
                {
                    const bool isSelected = (selected_image_index == i);
                    if (ImGui::Selectable((std::string(IMAGE_TOPICS.at(i))).c_str(), isSelected))
                    {
                        selected_image_index = i;
                    }
                    if (isSelected)
                    {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }

            if (selected_image_index < gl_textures.size() && selected_image_index < image_data.size())
            {
                auto &texture = gl_textures[selected_image_index];
                auto &[image, dirty] = image_data[selected_image_index];
                auto widget_size = ImGui::GetContentRegionAvail();

                auto image_aspect = static_cast<float>(image.Width) / static_cast<float>(image.Height);
                auto widget_aspect = widget_size.x / widget_size.y;

                ImVec2 display_size;
                if (widget_aspect > image_aspect)
                {
                    display_size = ImVec2(widget_size.y * image_aspect, widget_size.y);
                }
                else
                {
                    display_size = ImVec2(widget_size.x, widget_size.x / image_aspect);
                }

                ImGui::Image(static_cast<ImTextureID>(texture), display_size);
            }
        }
        ImGui::End();

        input.update();

        if (ImGui::Begin("Control"))
        {
            input.imgui_panel();

            ImGui::End();
        }

        controller.process(input);
        controller.transfer_data_ros();

        if (ImGui::Begin("Sensor Data"))
        {
            std::lock_guard<std::mutex> lock(barcode_mutex);
            ImGui::Text("Barcodes:");
            for(const auto &[code,has_code]: shared_barcodes) ImGui::Text("%s: %lu",code.c_str(),has_code);
            if(shared_barcodes.size()>0)
            {
                if(ImGui::Button("Export"))
                {
                    std::string barcodes;
                    for(const auto &pair: shared_barcodes)
                    {
                        std::string keyStr = pair.first;
                        std::string valueStr = std::to_string(pair.second);
                        std:: string line = keyStr + ": " + valueStr;
                        barcodes += line +"\n";
                    }
                    std::ofstream outputFile("Guiniverse_Barcode.txt");
                    outputFile << barcodes;
                    outputFile.close();
                }
            }
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

    for (auto &texture : gl_textures)
        glDeleteTextures(1, &texture);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    running = false;
}
