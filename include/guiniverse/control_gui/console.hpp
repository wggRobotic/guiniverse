#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class Console
{
public:
    Console();
    ~Console();

    void Init();
    void ImGuiPanel();

private:
    static void CaptureOutput();
    static int pipe_fd[2]; // Pipe for redirection
    static std::thread capture_thread;
    static std::atomic<bool> running;
    static bool auto_scroll;
};
