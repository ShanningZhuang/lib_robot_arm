
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
    auto tmotor4 = new lkMotorController(0x144, "lkmotor4", tmotor1->get_can_comm(), MotorType::lkMG5010);
    auto tmotor5 = new lkMotorController(0x145, "lkmotor5", tmotor1->get_can_comm(), MotorType::lkMG4005);
    auto tmotor6 = new lkMotorController(0x146, "lkmotor6", tmotor1->get_can_comm(), MotorType::lkMG4005);

    tmotor1->init_receive_buffer();
    tmotor2->init_receive_buffer();
    tmotor3->init_receive_buffer();
    tmotor4->init_receive_buffer();
    tmotor5->init_receive_buffer();
    tmotor6->init_receive_buffer();

    for (int i = 0; i <= 1000; i++)
    {
        tmotor1->get_multi_encoder();
        tmotor2->get_multi_encoder();
        tmotor3->get_multi_encoder();
        tmotor4->get_multi_encoder();
        tmotor5->get_multi_encoder();
        tmotor6->get_multi_encoder();
        sleep(1);
    }


    return 0;
}
