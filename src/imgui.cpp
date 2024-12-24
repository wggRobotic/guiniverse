#include <guiniverse/ros2_imgui_integration.hpp>

static void set_style()
{
    ImVec4 *colors = ImGui::GetStyle().Colors;
}

static void on_key(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
    {
        running = false;
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

// ImGui rendering thread
void imgui_thread()
{
    if (!glfwInit())
        return;

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    // glfwWindowHint(GLFW_MOUSE_PASSTHROUGH, GLFW_TRUE);
    // glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
    // glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
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
    // io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    // set_style();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Set up a clear color
    float clear_color[]{0.00f, 0.00f, 0.00f, 0.35f};
    bool show_styles = false;

    while (!glfwWindowShouldClose(window) && running)
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Begin main docking space
        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport()->ID, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        // Lock shared data for safe access
        std::string display_data;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            display_data = shared_data;
        }

        if (ImGui::Begin("Home"))
        {
            ImGui::Text("Shared Data: %s", display_data.c_str());
            ImGui::ColorEdit4("Clear Color", clear_color);
            ImGui::Checkbox("Show Styles", &show_styles);
        }
        ImGui::End();

        if (ImGui::Begin("Control"))
        {
        }
        ImGui::End();

        if (ImGui::Begin("Camera"))
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
