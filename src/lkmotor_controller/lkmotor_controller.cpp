#include <memory>
#include "lkmotor_controller.hpp"
#include "utils.hpp"

double clip_limit(std::array<double,2> lim, double value)
{
    if(value < lim[0])
    {
        value = lim[0];
        std::cout<<"lower than limit, set to limit"<<std::endl;
    }
    else if(value > lim[1])
    {
        value = lim[1];
        std::cout<<"higher than limit, set to limit"<<std::endl;
    }
    return value;
}

STATUS lkMotorController::send(std::vector<BYTE> &msg, uint32_t id)
{
    STATUS status = this->can_comm->send(id, msg);
    return status;
}
STATUS lkMotorController::receive(TPCANMsg &msg)
{
    STATUS status = this->can_comm->receive(msg);
    if (status)
        return FAIL;

    return SUCCESS;
}

STATUS lkMotorController::receive_timeout(TPCANMsg &msg, int ms)
{
    STATUS status = this->can_comm->receive_timeout(msg, ms);
    if (status)
        return FAIL;

    return SUCCESS;
}
lkMotorController::lkMotorController(uint32_t id, std::string motor_name, std::shared_ptr<CanCommunicator> can_comm_ptr, 
                                   MotorType motor_type)
    : id(id), motor_name(motor_name)
{
    this->config = new MotorConfig(motor_type);

    this->can_comm = can_comm_ptr;
}

lkMotorController::lkMotorController(uint32_t id, std::string motor_name, std::string device_path,  MotorType motor_type)
    : id(id), motor_name(motor_name), device_path(device_path)
{
    this->config = new MotorConfig(motor_type);

    uint16_t can_msg_type;
    STATUS status;

    this->can_comm = std::make_shared<CanCommunicator>(device_path, can_msg_type);

    status = this->can_comm->open();
    if (status)
        throw std::runtime_error("Failed to open CAN device");

    status = this->can_comm->init();
    if (status)
        throw std::runtime_error("Failed to initialize CAN device");
}

lkMotorController::~lkMotorController()
{
    STATUS status;
    this->close();

    status = this->can_comm->close();
    if (status)
        throw std::runtime_error("Failed to close CAN device");
}

std::shared_ptr<CanCommunicator> lkMotorController::get_can_comm()
{
    return this->can_comm;
}


STATUS lkMotorController::close()
{
    if (!this->is_open)
        return SUCCESS;
    std::vector<BYTE> msg = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    STATUS status = this->send(msg, this->id);
    if (status)
        return FAIL;
    this->is_open = false;
    return SUCCESS;
}

STATUS lkMotorController::open()
{
    if (this->is_open)
        return SUCCESS;
    std::vector<BYTE> msg = {0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    STATUS status = this->send(msg, this->id);
    if (status)
        return FAIL;
    this->is_open = true;
    return SUCCESS;
}

STATUS lkMotorController::stop()
{
    if (this->is_open)
        return SUCCESS;
    std::vector<BYTE> msg = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    STATUS status = this->send(msg, this->id);
    if (status)
        return FAIL;
    this->is_open = true;
    return SUCCESS;
}

STATUS lkMotorController::set_current(double double_current_control)
{
    std::vector<BYTE> send_msg = {0xA1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int16_t int_iqcontrol = (int16_t)(double_current_control/33*2048);
    send_msg[4] = *(uint8_t *)(&int_iqcontrol);
    send_msg[5] = *((uint8_t *)(&int_iqcontrol)+1);
    this->send(send_msg, this->id);
    TPCANMsg recv_msg;
    if (this->receive(recv_msg))
        return FAIL;
    this->process_feedback(recv_msg);
    return SUCCESS;
}

STATUS lkMotorController::set_multi_pos(double rad)
{
    rad = clip_limit(this->config->get_pos_lims(),rad);
    double degree = this->config->get_reduction_ratio()*rad*180/3.141592653589793;
    std::vector<BYTE> send_msg = {0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t angleControl = degree*100;
    send_msg[4] = *(uint8_t *)(&angleControl);
    send_msg[5] = *((uint8_t *)(&angleControl)+1);
    send_msg[6] = *((uint8_t *)(&angleControl)+2);
    send_msg[7] = *((uint8_t *)(&angleControl)+3);
    this->send(send_msg, this->id);
    TPCANMsg recv_msg;
    if (this->receive(recv_msg))
        return FAIL;
    return SUCCESS;
}

STATUS lkMotorController::set_multi_pos(double rad, double max_speed_rad)
{
    rad = clip_limit(this->config->get_pos_lims(),rad);
    double degree = this->config->get_reduction_ratio()*rad*180/3.141592653589793;
    std::vector<BYTE> send_msg = {0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t angleControl = degree*100;
    u_int16_t max_speed_degree = (u_int16_t)(this->config->get_reduction_ratio()*max_speed_rad*180/3.141592653589793);
    send_msg[2] = *(uint8_t *)(&max_speed_degree);
    send_msg[3] = *((uint8_t *)(&max_speed_degree)+1);
    send_msg[4] = *(uint8_t *)(&angleControl);
    send_msg[5] = *((uint8_t *)(&angleControl)+1);
    send_msg[6] = *((uint8_t *)(&angleControl)+2);
    send_msg[7] = *((uint8_t *)(&angleControl)+3);
    this->send(send_msg, this->id);
    TPCANMsg recv_msg;
    if (this->receive(recv_msg))
        return FAIL;
    this->process_feedback(recv_msg);
    return SUCCESS;
}

STATUS lkMotorController::set_single_pos(double degree,u_int8_t direction)
{
    //direction 0 is clock-wise direction 1 is anti-clock-wise
    std::vector<BYTE> send_msg = {0xA5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_msg[1] = direction;
    u_int32_t angleControl = degree*100;
    send_msg[4] = *(uint8_t *)(&angleControl);
    send_msg[5] = *((uint8_t *)(&angleControl)+1);
    send_msg[6] = *((uint8_t *)(&angleControl)+2);
    send_msg[7] = *((uint8_t *)(&angleControl)+3);
    this->send(send_msg, this->id);
}

STATUS lkMotorController::set_single_pos(double degree,u_int8_t direction,u_int16_t max_speed)
{
    //direction 0 is clock-wise direction 1 is anti-clock-wise
    std::vector<BYTE> send_msg = {0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    send_msg[1] = direction;
    u_int32_t angleControl = degree*100;
    send_msg[2] = *(uint8_t *)(&max_speed);
    send_msg[3] = *((uint8_t *)(&max_speed)+1);
    send_msg[4] = *(uint8_t *)(&angleControl);
    send_msg[5] = *((uint8_t *)(&angleControl)+1);
    send_msg[6] = *((uint8_t *)(&angleControl)+2);
    send_msg[7] = *((uint8_t *)(&angleControl)+3);
    this->send(send_msg, this->id);
}

STATUS lkMotorController::set_incre_pos(double degree)
{
    std::vector<BYTE> send_msg = {0xA6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t angleControl = degree*100;
    send_msg[4] = *(uint8_t *)(&angleControl);
    send_msg[5] = *((uint8_t *)(&angleControl)+1);
    send_msg[6] = *((uint8_t *)(&angleControl)+2);
    send_msg[7] = *((uint8_t *)(&angleControl)+3);
    this->send(send_msg, this->id);
}

STATUS lkMotorController::set_incre_pos(double degree,u_int16_t max_speed)
{
    std::vector<BYTE> send_msg = {0xA7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32_t angleControl = degree*100;
    send_msg[2] = *(uint8_t *)(&max_speed);
    send_msg[3] = *((uint8_t *)(&max_speed)+1);
    send_msg[4] = *(uint8_t *)(&angleControl);
    send_msg[5] = *((uint8_t *)(&angleControl)+1);
    send_msg[6] = *((uint8_t *)(&angleControl)+2);
    send_msg[7] = *((uint8_t *)(&angleControl)+3);
    this->send(send_msg, this->id);
}

STATUS lkMotorController::get_encoder()
{
    std::vector<BYTE> send_msg = {0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this->send(send_msg, this->id);
    TPCANMsg can_msg;
    std::vector<BYTE> msg;
    if (this->receive(can_msg))
        return FAIL;
    for (auto item : can_msg.DATA)
    {
        msg.push_back(item);
    }
    uint16_t encoderPos = (msg[3] << 8) | (msg[2] );
    uint16_t encoderRaw = (msg[5] << 8) | (msg[4] );
    uint16_t encoderOffset = (msg[7] << 8) | (msg[6] );
    std::cout<<"encoderPos: "<<encoderPos<<"  encoderRaw: "<<encoderRaw<<"  encoderOffset: "<<encoderOffset<<std::endl;
}

double lkMotorController::get_multi_encoder()
{
    std::vector<BYTE> send_msg = {0x92, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this->send(send_msg, this->id);
    TPCANMsg can_msg;
    std::vector<BYTE> msg;
    if (this->receive(can_msg))
        return FAIL;

    for (auto item : can_msg.DATA)
    {
        msg.push_back(item);
    }
    int64_t multi_angle_int = (((u_int64_t)msg[7]) << 56) |(((u_int64_t)msg[6]) << 48) |(((u_int64_t)msg[5]) << 40)|(((u_int64_t)msg[4]) << 32) |(((u_int64_t)msg[3]) << 24) |(((u_int64_t)msg[2]) << 16) | (((u_int64_t)msg[1]) << 8);
    multi_angle_int = multi_angle_int>>8;
    double multi_angle = multi_angle_int*0.01;
    // std::cout<<"motor: "<<(this->id-0x140)<<" "<<can_msg.ID<<" multi_angel_double: "<<multi_angle<<" multi_rad: "<<3.141592653589793*(multi_angle/180)/this->config->get_reduction_ratio()<<std::endl;
    
    return 3.141592653589793*(multi_angle/180)/this->config->get_reduction_ratio();
}

STATUS lkMotorController::get_single_encoder()
{
    std::vector<BYTE> send_msg = {0x94, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this->send(send_msg, this->id);
    TPCANMsg can_msg;
    std::vector<BYTE> msg;
    if (this->receive(can_msg))
        return FAIL;

    for (auto item : can_msg.DATA)
    {
        msg.push_back(item);
    }
    u_int32_t single_angle_uint = (msg[7] << 24) |u_int64_t(msg[6] << 16) |u_int64_t(msg[5] << 8) |u_int64_t(msg[4]);
    for(int i=0;i<8;i++)
    {
        std::cout<<i<<" :"<<std::hex<<int(msg[i])<<"  ";
    }
    double single_angle = single_angle_uint*0.01;
    std::cout<<"motor: "<<(this->id-0x140)<<" singel angel: "<<single_angle<<std::endl;
}

STATUS lkMotorController::set_zero2ROM()
{
    std::vector<BYTE> send_msg = {0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this->send(send_msg, this->id);
}


STATUS lkMotorController::get_status(MotorStatus &motor_status)
{

    TPCANMsg can_msg;
    std::vector<BYTE> msg;

    if (this->receive(can_msg))
        return FAIL;

    for (auto item : can_msg.DATA)
    {
        msg.push_back(item);
    }

    int id = msg[0];

    int pos_int = (msg[1] << 8) | msg[2];
    int vel_int = (msg[3] << 4) | (msg[4] >> 4);
    int cur_int = ((msg[4] & 0xF) << 8) | msg[5];
    int temp_int = msg[6];
    int flag_int = msg[7];

    motor_status.id = id;
    motor_status.pos = uint_to_double(pos_int, this->config->get_pos_lims()[1], this->config->get_pos_lims()[0], 16);
    motor_status.vel = uint_to_double(vel_int, this->config->get_vel_lims()[1], this->config->get_vel_lims()[0], 12);
    motor_status.current = uint_to_double(cur_int, this->config->get_tor_lims()[1], this->config->get_tor_lims()[0], 12);
    motor_status.tempature = double(temp_int) - 40;
    motor_status.error_flag = flag_int;

    return SUCCESS;
}

STATUS lkMotorController::init_receive_buffer()
{
    TPCANMsg can_msg;
    int timeout_ms = 100;

    for (auto i = 0; i < 10; i++)
        STATUS status = this->receive_timeout(can_msg, timeout_ms);

    return SUCCESS;
}

STATUS lkMotorController::set_encoder_zero()
{
    std::vector<BYTE> send_msg = {0x95, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    this->send(send_msg, this->id);
}

void lkMotorController::process_feedback(TPCANMsg can_msg)
{
    std::vector<BYTE> msg;
    int id = this->id-0x140;
    for (auto item : can_msg.DATA)
    {
        msg.push_back(item);
    }
    int16_t current_value = (((u_int8_t)msg[2]) ) | (((u_int8_t)msg[3])<< 8);
    int16_t velocity_value = (((u_int8_t)msg[4]) ) | (((u_int8_t)msg[5])<< 8);
    uint16_t encoder_value = (((u_int8_t)msg[6])) | (((u_int8_t)msg[7])<< 8);
    // std::cout<<"motor: "<<(this->id-0x140)<<" current value: "<<current_value<<" velocity_value: "<<velocity_value<<" encoder_value: "<<encoder_value<<std::endl;
}