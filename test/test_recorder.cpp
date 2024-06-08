#include <chrono>
#include <thread>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <time.h>

#include "lkmotor_controller.hpp"
#include "utils.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <signal.h>
#include <fstream>
std::atomic<bool> quit(false);

void sig_handler(int signum)
{
  if (signum == SIGINT)
    quit.store(true);
}
int main()
{   
    signal(SIGINT, &sig_handler);
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

    lkMotorController* pointer_lkMotorController[6] = {tmotor1,tmotor2,tmotor3,tmotor4,tmotor5,tmotor6};
    double joint_state[6];


    YAML::Node total;  // starts out as null
    YAML::Node each;  // starts out as null
    std::this_thread::sleep_for(std::chrono::milliseconds(10000)); // 等待100毫秒
    tmotor1->set_current(0);
    tmotor2->set_current(0);
    tmotor3->set_current(0);
    tmotor4->set_current(0);
    tmotor5->set_current(0);
    tmotor6->set_current(0);
    while(!quit)
    {
        YAML::Node each;  // starts out as null
        for(int i = 0; i < 6; i++) {
            joint_state[i] = pointer_lkMotorController[i]->get_multi_encoder();
            each.push_back(joint_state[i]);
        }
        total["joint_traj"].push_back(each);
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 等待100毫秒
    }

    std::ofstream fout("/home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/trajectory/test.yaml");
    fout << total;
    std::cout<<"done"<<std::endl;
    return 0;
}
