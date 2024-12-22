#include <guiniverse/ros2_imgui_integration.hpp>

static void set_style()
{
    ImVec4 *colors = ImGui::GetStyle().Colors;
}

static void on_key(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_RELEASE)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// ImGui rendering thread
void imgui_thread() {
    if (!glfwInit())
        return;

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    GLFWwindow* window = glfwCreateWindow(800, 600, "GUINIVERSE", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glEnable(GL_MULTISAMPLE);
    glewInit();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    float clear_color[]{0.00f, 0.00f, 0.00f, 0.35f};
    bool show_styles = false;

    while (!glfwWindowShouldClose(window) && running) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ImGui layout
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            ImGui::Begin("Home");
            ImGui::Text("Shared Data: %s", shared_data.c_str());
            ImGui::ColorEdit4("Clear Color", clear_color);
            ImGui::Checkbox("Show Styles", &show_styles);
            ImGui::End();
        }

        ImGui::Render();
        glClearColor(clear_color[0], clear_color[1], clear_color[2], clear_color[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    running = false; // Notify other threads to stop
}

