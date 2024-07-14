#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H
#include <iostream>
#include <string>
//belajar git
class PIDController
{
public:
    // Constructor
    PIDController();

    // Fungsi Utama
    void init(int mode, float KP, float KI, float KD, bool is_active);
    double compute_action(double target, double feedback, float samplingTime);
    void setActive(bool command_active);

    // Fungsi Utility
    double clamp(double val, double min, double max);

    // Variabel
    float errorIntegral, errorDerivative;
    double err,targett;
    double out;

private:

    double Kp, Ki, Kd;

    double prev_err_[2], prev_out_[2];
    double prev_ril_err[3];
    int motor_num;
    bool is_active_;
};

#endif