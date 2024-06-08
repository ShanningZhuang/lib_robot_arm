#pragma once

#include <string>
#include <vector>
#include <memory>

#include "../can_communicator/can_communicator.hpp"
#include "motor_config.hpp"

typedef struct motor_status
{
    uint32_t id;
    double pos;
    double vel;
    double current;
    double tempature;
    uint32_t error_flag;
} MotorStatus;

typedef enum
{
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT,
    CAN_PACKET_SET_CURRENT_BRAKE,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_ORIGIN_HERE,
    CAN_PACKET_SET_POS_SPD,
} CAN_PACKET_ID;


class lkMotorController
{
public:
    //static member declare
    std::shared_ptr<CanCommunicator> can_comm;
    const std::string motor_name;
    const uint32_t id;
    const std::string device_path;
    bool is_open = false;
    MotorConfig *config;
    MotorStatus motor_status_servo; // Just for Servo mode

public:
    //can conmunicate method
    STATUS send(std::vector<BYTE> &msg, uint32_t id);
    STATUS receive(TPCANMsg &msg);
    STATUS receive_timeout(TPCANMsg &msg, int ms);

public:
    //construct&deconstruct
    lkMotorController(uint32_t id, std::string motor_name, std::string device_path,  MotorType motor_type);
    lkMotorController(uint32_t id, std::string motor_name, std::shared_ptr<CanCommunicator> can_comm_ptr, 
                     MotorType motor_type);
    ~lkMotorController();

    // Common method
    std::shared_ptr<CanCommunicator> get_can_comm();
    uint32_t get_id();
    uint32_t get_ext_id(uint32_t id, uint32_t packet_id);



    STATUS init_receive_buffer();
    STATUS get_status(MotorStatus &motor_status);

    void process_feedback(TPCANMsg can_msg);

    STATUS close();//command 1
    STATUS open();//command 2
    STATUS stop();//command 3
    //command 4 not used in our motor
    STATUS set_current(double double_current_control);// command 5 current_control
    //lack command 6 set_velocity
    STATUS set_multi_pos(double rad);// command 7 multi_pos no limit
    STATUS set_multi_pos(double rad,double max_speed_rad);// command 8 multi_pos limit speed
    STATUS set_single_pos(double degree,u_int8_t direction);// command 9 single_pos no limit
    STATUS set_single_pos(double degree,u_int8_t direction,u_int16_t max_speed);// command 10 single_pos limit speed
    STATUS set_incre_pos(double degree);// command 11 incre_pos no limit
    STATUS set_incre_pos(double degree,u_int16_t max_speed);// command 12 incre_pos limit speed
    STATUS get_pid_para();// command 13 read pid para
    //lack command 14 set_pid_para2RAM
    //lack command 15 set_pid_para2ROM
    // lack command 16 read acceleration(usage?)
    // lack command 17 set Acceleration2RAM acceleration(usage?)
    STATUS get_encoder();// command 18 get_encoder_pos
    STATUS set_encoder_zero();//command 19 set encoder offset
    STATUS set_zero2ROM();//command 20 set_current_pos2zero2ROM
    double get_multi_encoder();// command 21 read_multi_encoder
    STATUS get_single_encoder();// command 22 read_single_encoder
    //lack command 23 no use
    //lack command 24 read state1
    //lack command 25 erase error
    //lack command 26 read state2
    //lack command 27 read state3
    
    
};
