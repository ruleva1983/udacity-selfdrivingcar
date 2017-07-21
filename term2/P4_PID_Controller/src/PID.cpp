#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    i_error = 0;
    d_error = 0;
    p_error = 0;
}

void PID::UpdateError(double cte) {
    i_error += cte;
    d_error = cte - p_error;
    p_error = cte;

}

double PID::TotalError() {
    double steering_angle;
    steering_angle = -Kp * p_error + Ki * i_error - Kd * d_error;
    if (steering_angle > 1.0)
        return 1.0;
    else if (steering_angle < -1.0)
        return -1.0;
    else
        return steering_angle;
}

