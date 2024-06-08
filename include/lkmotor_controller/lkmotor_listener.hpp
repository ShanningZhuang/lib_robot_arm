#pragma once
#include <thread>
#include <atomic>
#include <vector>

#include "motor_controller.hpp"

class MotorListener
{
public:
    std::vector<MotorController *> motor_list;
    std::atomic<bool> is_running{false};
    std::thread *listener_thread;

public:
    MotorListener();
    ~MotorListener();

    void add_motor(MotorController *motor);
    void start();
    void stop();
};

void receive_data(std::vector<MotorController *> *motor_list, std::atomic<bool> *is_running);
void update_motor_status(std::vector<MotorController *> *motor_list, MotorStatus motor_status);
