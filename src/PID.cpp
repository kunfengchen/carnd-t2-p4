#include <iostream>

#include "PID.h"

/// using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    isFirstUpdate = true;
}

void PID::UpdateError(double cte) {
    if (isFirstUpdate) {
        pre_cte = cte;
        pre_time = std::clock();
        isFirstUpdate = false;
    }
    cur_time = std::clock();
    p_error = -Kp * cte;

    dif_cte = cte - pre_cte;
    dt = (cur_time - pre_time) * 1.0 / CLOCKS_PER_SEC; // delta time per second
    std::cout << "dt= " << dt << " : " << cur_time << "-" << pre_time << std::endl;
    if (dt < 0.00001) {
        d_error = 0;
    } else {
        d_error = -Kd * dif_cte / dt;
    }

    pre_cte = cte;
    pre_time = cur_time;

    int_cte += cte;
    i_error = -Ki * int_cte;
    std::cout << "pe=" << p_error << " ie=" << i_error << " de=" << d_error << std::endl;
}

double PID::TotalError() {
    // return p_error;
    // return p_error + d_error;
    return p_error + d_error + i_error;
}

