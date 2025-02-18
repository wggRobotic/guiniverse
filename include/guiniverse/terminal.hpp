#pragma once
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>

class ImGuiTerminal {
public:
    ImGuiTerminal();
    ~ImGuiTerminal();

    void Init();
    void Draw();
    void Log(const char* text);

private:
    static void CaptureOutput();
    static int pipe_fd[2]; // Pipe for redirection
    static std::thread capture_thread;
    static std::atomic<bool> running;
    static bool auto_scroll;
};