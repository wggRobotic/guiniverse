#include "guiniverse/ros2_imgui_integration.hpp"
#include "second_file.cpp"

// Shared data between threads
std::atomic<bool> running(true);
std::mutex data_mutex;
std::string shared_data = "Hello from ROS2!";

// ROS2 main thread
void ros2_main_thread() {
    auto node = rclcpp::Node::make_shared("ros2_thread");
    rclcpp::Rate rate(10);

    while (rclcpp::ok() && running) {
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            shared_data = "ROS2 is running: " + std::to_string(rclcpp::Clock().now().seconds());
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

// ImGui rendering thread
void imgui_thread() {
    if (!glfwInit()) return;

    GLFWwindow* window = glfwCreateWindow(800, 600, "ImGui + ROS2", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);
    glewInit();

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Set up a clear color
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    signal(SIGINT, [](int) { // Handle SIGINT (Ctrl+C)
        running = false;
    });

    while (!glfwWindowShouldClose(window) && running) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Begin main docking space
        ImGui::DockSpaceOverViewport(ImGui::GetMainViewport()->ID);

        // Lock shared data for safe access
        std::string display_data;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            display_data = shared_data;
        }

        // ImGui window
        ImGui::Begin("ROS2 Data");
        ImGui::Text("Shared Data: %s", display_data.c_str());
        ImGui::End();

        ImGui::Render();
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

int main(int argc, char** argv) {
    printMessage();
    rclcpp::init(argc, argv);

    std::thread ros_thread(ros2_main_thread);
    std::thread gui_thread(imgui_thread);

    ros_thread.join();
    gui_thread.join();

    rclcpp::shutdown();
    return 0;
}