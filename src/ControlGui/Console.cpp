#include <guiniverse/ControlGui/Console.hpp>
#include <imgui.h>
#include <unistd.h>   // For pipe, dup2
#include <fcntl.h>    // For fcntl
#include <stdio.h>    // For stdout, stderr
#include <sstream>

std::vector<std::string> log_lines;
std::mutex log_mutex;
int Console::pipe_fd[2];
std::thread Console::capture_thread;
std::atomic<bool> Console::running{false};
bool Console::auto_scroll = false;

Console::Console() {}

Console::~Console() {
    running = false;
    if (capture_thread.joinable()) {
        capture_thread.join();
    }
}

void Console::Init() {
    if (pipe(pipe_fd) == -1) {
        perror("pipe failed");
        return;
    }

    // Set non-blocking mode
    fcntl(pipe_fd[0], F_SETFL, O_NONBLOCK);

    // Redirect stdout and stderr
    dup2(pipe_fd[1], STDOUT_FILENO);
    dup2(pipe_fd[1], STDERR_FILENO);

    // Disable buffering for stdout and stderr
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    running = true;
    capture_thread = std::thread(CaptureOutput);
}

void Console::CaptureOutput() {
    char buffer[512];
    while (running) {
        ssize_t count = read(pipe_fd[0], buffer, sizeof(buffer) - 1);
        if (count > 0) {
            buffer[count] = '\0';
            std::lock_guard<std::mutex> lock(log_mutex);
            log_lines.emplace_back(buffer);
        }
    }
}

void Console::ImGuiPanel() {
    ImGui::Begin("Console");

    ImGui::BeginChild("ScrollingRegion", ImVec2(0, -ImGui::GetTextLineHeightWithSpacing()), false, ImGuiWindowFlags_HorizontalScrollbar);


    std::lock_guard<std::mutex> lock(log_mutex);
    for (const auto& line : log_lines) {
        ImGui::TextUnformatted(line.c_str());
    }

    if (auto_scroll) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
    ImGui::End();
}
