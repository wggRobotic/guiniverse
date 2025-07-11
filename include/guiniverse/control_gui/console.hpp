#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

class Console final
{
public:
    Console();
    ~Console();

    void Init();
    void ImGuiPanel();

protected:
    void CaptureOutput();

private:
    std::vector<std::string> m_LogLines;
    std::mutex m_LogMutex;
    int m_PipeFd[2];
    std::thread m_CaptureThread;
    std::atomic<bool> m_Running;
    bool m_AutoScroll;
};
