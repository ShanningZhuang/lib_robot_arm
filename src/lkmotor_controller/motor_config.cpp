#include "motor_config.hpp"

MotorConfig::MotorConfig(MotorType motor_type)
{
  this->motor_type = motor_type;

  switch (this->motor_type)
  {
    case lkMG4005:
      this->pos_lims = { -0.0, 0.0 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0 };
      this->reduction_ratio = 10;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;

    case lkMG5010:
      this->pos_lims = { -0.0, 0.0 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0 };
      this->reduction_ratio = 10;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;

    case lkMG6012:
      this->pos_lims = { -0.0, 0.0 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0 };
      this->reduction_ratio = 36;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
    case lkMG6012J1:
      this->pos_lims = { -1.5, 1.5 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0 };
      this->reduction_ratio = 36;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
    case lkMG6012J2:
      this->pos_lims = { -1.8, 0.8 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0 };
      this->reduction_ratio = 36;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
    case lkMG6012J3:
      this->pos_lims = { -0.0, 2.9 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0 };
      this->reduction_ratio = 36;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
    case lkMG5010J4:
      this->pos_lims = { -1.6, -0.15 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0 };
      this->reduction_ratio = 10;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
    case lkMG4005J5:
      this->pos_lims = { -0.5, 1.8 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0.0 };
      this->reduction_ratio = 10;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
    case lkMG4005J6:
      this->pos_lims = { -6.28, 6.28 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0.0 };
      this->reduction_ratio = 10;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      
      break;
    case Reduct36NOlimit:
      this->pos_lims = { -100, 100 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0.0 };
      this->reduction_ratio = 36;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
    case Reduct10NOlimit:
      this->pos_lims = { -100, 100 };
      this->vel_lims = { -0.0, 0.0 };
      this->tor_lims = { -0.0, 0.0 };
      this->kp_lims = { 0.0, 0.0 };
      this->kd_lims = { 0.0, 0.0 };
      this->cur_lims = { 0.0, 0.0 };
      this->reduction_ratio = 10;
      this->torque_constant = 0.0;
      this->pole_num = 0;
      break;
      
    default:
      throw "Unsupported motor type";
      break;
  }
}
