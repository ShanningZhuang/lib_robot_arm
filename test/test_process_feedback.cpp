
#include <unistd.h>
#include <iostream>
#include <vector>
#include <time.h>

#include "lkmotor_controller.hpp"
#include "utils.hpp"

int main()
{   
    auto tmotor1 = new lkMotorController(0x141, "lkmotor1", "/dev/pcan32", MotorType::lkMG6012J1);
    auto tmotor2 = new lkMotorController(0x142, "lkmotor2", tmotor1->get_can_comm(),MotorType::lkMG6012J2);
    auto tmotor3 = new lkMotorController(0x143, "lkmotor3", tmotor1->get_can_comm(),MotorType::lkMG6012J3);
    auto tmotor4 = new lkMotorController(0x144, "lkmotor4", tmotor1->get_can_comm(), MotorType::lkMG5010J4);
    auto tmotor5 = new lkMotorController(0x145, "lkmotor5", tmotor1->get_can_comm(), MotorType::lkMG4005J5);
    auto tmotor6 = new lkMotorController(0x146, "lkmotor6", tmotor1->get_can_comm(), MotorType::lkMG4005J6);

    tmotor1->init_receive_buffer();
    tmotor2->init_receive_buffer();
    tmotor3->init_receive_buffer();
    tmotor4->init_receive_buffer();
    tmotor5->init_receive_buffer();
    tmotor6->init_receive_buffer();
    double speed_limit = 0.2;
    tmotor1->set_multi_pos(0.0,speed_limit);
    tmotor2->set_multi_pos( -0.3643,speed_limit);
    tmotor3->set_multi_pos(2.7361,speed_limit);
    tmotor4->set_multi_pos(-0.8626,speed_limit);
    tmotor5->set_multi_pos(0,speed_limit);
    tmotor6->set_multi_pos(0,speed_limit);
    sleep(5);
    while(1)
    {
        // tmotor1->set_multi_pos(0.0,speed_limit);
        // tmotor2->set_multi_pos( -0.3643,speed_limit);
        // tmotor3->set_multi_pos(2.7361,speed_limit);
        // tmotor4->set_multi_pos(-0.8626,speed_limit);
        // tmotor5->set_multi_pos(0,speed_limit);
        // tmotor6->set_multi_pos(0,speed_limit);
        // tmotor6->set_current(0);
        sleep(1);
    }
    return 0;
}
