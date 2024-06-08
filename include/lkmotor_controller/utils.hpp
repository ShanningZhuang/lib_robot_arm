#pragma once
#include <stdint.h>
#include <vector>
#include "libpcan.h"

double clip(double val, double min_value, double max_value);
u_int32_t double_to_uint(double val, double max_val, double min_val, u_int32_t bit_width);
double uint_to_double(u_int32_t val, double max_val, double min_val, u_int32_t bit_width);
std::vector<BYTE> double_to_4bytes(double val);
int32_t rps2int(double rad_s,double dps_lsb);