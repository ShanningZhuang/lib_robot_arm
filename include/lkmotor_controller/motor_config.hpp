#include <string>
#include <array>

#define LIMS std::array<double, 2>

enum MotorType
{
  lkMG4005,
  lkMG5010,
  lkMG6012,
  lkMG6012J1,
  lkMG6012J2,
  lkMG6012J3,
  lkMG5010J4,
  lkMG4005J5,
  lkMG4005J6,
  Reduct36NOlimit,
  Reduct10NOlimit,
};

class MotorConfig
{
  MotorType motor_type;
  LIMS pos_lims;
  LIMS vel_lims;
  LIMS tor_lims;
  LIMS kp_lims;
  LIMS kd_lims;
  LIMS cur_lims;
  double torque_constant;
  uint32_t pole_num;
  double reduction_ratio;

public:
  MotorConfig(MotorType motor_type);

  MotorType get_motor_type()
  {
    return this->motor_type;
  }
  LIMS get_pos_lims()
  {
    return this->pos_lims;
  }
  LIMS get_vel_lims()
  {
    return this->vel_lims;
  }
  LIMS get_tor_lims()
  {
    return this->tor_lims;
  }
  LIMS get_kp_lims()
  {
    return this->kp_lims;
  }
  LIMS get_kd_lims()
  {
    return this->kd_lims;
  }
  double get_reduction_ratio()
  {
    return this->reduction_ratio;
  }
  uint32_t get_pole_num()
  {
    return this->pole_num;
  }
  LIMS get_cur_lims()
  {
    return this->kd_lims;
  }
};
