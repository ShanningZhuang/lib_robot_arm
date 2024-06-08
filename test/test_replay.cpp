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
    auto tmotor1 = new lkMotorController(0x141, "lkmotor1", "/dev/pcan32", MotorType::Reduct36NOlimit);
    auto tmotor2 = new lkMotorController(0x142, "lkmotor2", tmotor1->get_can_comm(),MotorType::Reduct36NOlimit);
    auto tmotor3 = new lkMotorController(0x143, "lkmotor3", tmotor1->get_can_comm(),MotorType::Reduct36NOlimit);
    auto tmotor4 = new lkMotorController(0x144, "lkmotor4", tmotor1->get_can_comm(), MotorType::Reduct10NOlimit);
    auto tmotor5 = new lkMotorController(0x145, "lkmotor5", tmotor1->get_can_comm(), MotorType::Reduct10NOlimit);
    auto tmotor6 = new lkMotorController(0x146, "lkmotor6", tmotor1->get_can_comm(), MotorType::Reduct10NOlimit);

    tmotor1->init_receive_buffer();
    tmotor2->init_receive_buffer();
    tmotor3->init_receive_buffer();
    tmotor4->init_receive_buffer();
    tmotor5->init_receive_buffer();
    tmotor6->init_receive_buffer();
    double speed_limit = 0.2;

    lkMotorController* pointer_lkMotorController[6] = {tmotor1,tmotor2,tmotor3,tmotor4,tmotor5,tmotor6};
    double joint_state[6];
    double joint_pos = 0;
    int num=0;
    YAML::Node total = YAML::LoadFile("/home/zsn/SynologyDrive/drive_file/research/1_project/1_doing/robot_arm_design/lib_robot_arm/trajectory/test.yaml");
    YAML::Node each;  // starts out as null

    int len = total["joint_traj"].size();
    std::cout<<"Input anything to replay"<<std::endl;
    char input;
    std::cin>>input;
    each = total["joint_traj"][0];
    for(int i = 0; i < 6; i++) {
      joint_pos = each[i].as<double>();
      pointer_lkMotorController[i]->set_multi_pos(joint_pos, speed_limit);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    std::cout<<"Input anything to replay"<<std::endl;
    std::cin>>input;

    while(!quit&&num<len)
    {
        each = total["joint_traj"][num];
        for(int i = 0; i < 6; i++) {
        joint_pos = each[i].as<double>();
        pointer_lkMotorController[i]->set_multi_pos(joint_pos, speed_limit);
        }
        num++;
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 等待100毫秒
    }
    return 0;
}
