#include "motor_listener.hpp"

void MotorListener::add_motor(MotorController *motor)
{
    this->motor_list.push_back(motor);
}

void MotorListener::start()
{
    if (this->is_running.load())
        return;

    listener_thread = new std::thread(receive_data, &this->motor_list, &this->is_running);

    this->is_running.store(true);
}

void MotorListener::stop()
{
    if (this->is_running.load())
        this->is_running.store(false);
}

MotorListener::MotorListener()
{
}

MotorListener::~MotorListener()
{
    if (this->is_running.load())
        this->stop();
}

void receive_data(std::vector<MotorController *> *motor_list, std::atomic<bool> *is_running)
{
    MotorStatus motor_status;
    while (is_running->load())
    {
        for (auto &motor : *motor_list)
        {
            motor->get_status_servo(motor_status);
            update_motor_status(motor_list, motor_status);
        }
    }
}

void update_motor_status(std::vector<MotorController *> *motor_list, MotorStatus motor_status)
{
    for (auto &motor : *motor_list)
    {
        if (motor->get_id() == motor_status.id)
        {
            motor->set_motor_status(motor_status);
            break;
        }
    }
}
