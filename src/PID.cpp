#include <iostream>
#include "PID.h"
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    PID::Kp = Kp;
    PID::Ki = Ki;
    PID::Kd = Kd;

    PID::prev_cte = 0;
    PID::int_cte = 0;
    PID::n = 0;
    PID::err = 0;
}

double PID::UpdateError(double cte) {
    n++;
    double diff_cte = cte - prev_cte;
    prev_cte = cte;
    int_cte += cte;
    err += (1 + fabs(cte)) * (1 + fabs(cte));

    double steer = -Kp * cte - Kd * diff_cte - Ki * int_cte;
    if (steer > 0.6) steer = 0.6;
    if (steer < -0.6) steer = -0.6;

    //std::cout << "CTE: " << cte << " diff_cte: " << diff_cte << " int_cte: " << int_cte << std::endl;

    return steer;
}

double PID::TotalError() {
    return err / n;
}

