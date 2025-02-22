#include <guiniverse/ControlGui/ControlGui.hpp>

#include <thread>

ControlGui::ControlGui(int RobotControllerRefreshRate) : m_RobotControllerRefreshRate(RobotControllerRefreshRate) {
    m_Console.Init();
}

void ControlGui::addController(std::shared_ptr<RobotController> Controller)
{
    if (Controller == NULL) return; 

    m_Controllers.push_back(Controller);
}

void ControlGui::run()
{

    m_Running.store(true);

    std::thread robot_controller_thread(&ControlGui::RobotControllerThreadFunction, this);

    GuiFunction();

    robot_controller_thread.join();
}

void ControlGui::stop()
{
    m_Running.store(false);
}

