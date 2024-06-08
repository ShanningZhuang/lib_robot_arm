#include <algorithm>

#include "utils.hpp"
#define PI 3.1415926535
double clip(double val, double min_value, double max_value)
{
    val = std::max(val, min_value);
    val = std::min(val, max_value);
    return val;
}

u_int32_t double_to_uint(double val, double max_val, double min_val, u_int32_t bit_width)
{
    double range = max_val - min_val;

    return (u_int32_t)((val - min_val) * ((double)((1 << bit_width) / range)));
}

double uint_to_double(u_int32_t val, double max_val, double min_val, u_int32_t bit_width)
{
    double range = max_val - min_val;

    return ((double)val) * range / ((double)((1 << bit_width) - 1)) + min_val;
}

std::vector<BYTE> double_to_4bytes(double val)
{
    std::vector<BYTE> bytes;
    int32_t val_uint = (int32_t)val;

    for (int i = 0; i < 4; i++)
    {
        bytes.push_back((val_uint >> (8 * (3 - i))) & 0xFF);
    }

    return bytes;
}
int32_t rps2int(double rad_s,double dps_lsb)
{
    double dps = rad_s*180/PI;
    int32_t int_output = int32_t(dps/dps_lsb);
    return int_output;
}
