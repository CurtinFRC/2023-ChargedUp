#include "ControlUtil.h"

double wom::deadzone(double val, double deadzone) {
    return std::fabs(val) > deadzone ? val : 0;
}

double wom::spow2(double val) {
    return val*val*(val > 0 ? 1 : -1);
}