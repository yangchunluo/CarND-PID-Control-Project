#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double kp, double ki, double kd) {
    ClearErrors();
    SetParams(kp, ki, kd);
}

void PID::ClearErrors() {
    first_time = true;
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

void PID::SetParams(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

PID::~PID() {}

void PID::UpdateError(double cte) {
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

