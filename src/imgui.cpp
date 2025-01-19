#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <guiniverse/shared_data.hpp>
#include <imgui.h>

static int selected_image_index = 0;
static std::vector<ImageData> image_data;
static std::vector<GLuint> textures;

static void swap_buffers()
{
    std::lock_guard<std::mutex> lock(image_mutex);
    image_data = shared_image_data;
    textures.resize(image_data.size());
}

static void set_style()
{
    ImVec4 *colors = ImGui::GetStyle().Colors;
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

static void update_texture(GLuint texture, const cv::Mat &mat)
{
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mat.cols, mat.rows, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

static void refresh_image_data()
{
    auto &texture = textures[selected_image_index];
    auto &[mat, dirty] = image_data[selected_image_index];

    bool no_texture = !texture;
    if (no_texture)
        init_texture(texture, mat.cols, mat.rows);

    if (dirty || no_texture)
        update_texture(texture, mat);

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

        refresh_image_data();

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
            if (ImGui::BeginCombo("Select Image", ("Image " + std::to_string(selected_image_index)).c_str()))
            {

                for (size_t i = 0; i < image_data.size(); i++)
                {
                    const bool isSelected = (selected_image_index == i);
                    if (ImGui::Selectable(("Image " + std::to_string(i)).c_str(), isSelected))
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

            if (selected_image_index < image_data.size())
            {
                auto &texture = textures[selected_image_index];
                auto &[mat, dirty] = image_data[selected_image_index];
                auto widget_size = ImGui::GetContentRegionAvail();

                auto image_aspect = static_cast<float>(mat.cols) / mat.rows;
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

        swap_buffers();

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
