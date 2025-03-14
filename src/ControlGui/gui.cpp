#include <guiniverse/ControlGui/ControlGui.hpp>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <imgui.h>

void ControlGui::GuiFunction() {

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    auto window = glfwCreateWindow(800, 600, "GUINIVERSE", nullptr, nullptr);
    if (!window) return;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glEnable(GL_MULTISAMPLE);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    auto &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    float clear_color[]{0.00f, 0.00f, 0.00f, 0.35f};
    bool show_styles = false;

    for (int i = 0; i < m_Controllers.size(); i++)
        m_Controllers.at(i)->onGuiStartup();


    while (!glfwWindowShouldClose(window) && m_Running.load())
    {

        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        int robot_selected = m_RobotSelected.load();

        if (ImGui::Begin("Home"))
        {

            ImGui::Text("ROS2 is running %s", std::to_string(m_RosClock.now().seconds()).c_str());
            ImGui::ColorEdit4("Clear Color", clear_color);
            ImGui::Checkbox("Show Styles", &show_styles);
            
            if (ImGui::BeginCombo("Select which robot to control", robot_selected == NO_ROBOT_SELECTED ? "No robot selected" : m_Controllers.at(robot_selected)->getRobotName()))
            {
                if (ImGui::Selectable("No robot selected", robot_selected == NO_ROBOT_SELECTED))
                    robot_selected  = NO_ROBOT_SELECTED;

                for (int i = 0; i < m_Controllers.size(); i++)
                {
                    if (ImGui::Selectable(m_Controllers.at(i)->getRobotName(), robot_selected == i)) 
                        robot_selected = i;
                }
                
                ImGui::EndCombo();
            }

            m_RobotSelected.store(robot_selected);

        }
        ImGui::End();

        m_JoystickInput.update();
        m_JoystickInput.ImGuiPanel("Input");

        if (robot_selected != NO_ROBOT_SELECTED)
        {
            m_Controllers.at(robot_selected)->onGuiFrame(window, m_JoystickInput);
        }
        

        if (show_styles)
        {
            if (ImGui::Begin("Styles", &show_styles))
            {
                ImGui::ShowStyleEditor();

            }
            ImGui::End();
                
            
        }

        m_Console.ImGuiPanel();

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

    for (int i = 0; i < m_Controllers.size(); i++)
        m_Controllers.at(i)->onGuiShutdown();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);

    m_Running.store(false);
}