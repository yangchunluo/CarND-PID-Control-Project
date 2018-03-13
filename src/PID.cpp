#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double kp, double ki, double kd): Kp(kp), Ki(ki), Kd(kd) {
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

PID::~PID() {}

void PID::UpdateError(double cte) {
    static bool first_time = true;
    if (first_time) {
        d_error = 0;
        first_time = false;
    } else {
        d_error = cte - p_error; // p_error is the last invocation's CTE
    }
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    return p_error * Kp + i_error * Ki + d_error * Kd;
}

