#include "PIDController.h"


PIDController::PIDController()
{
    //INI MUNGKIN GUNA
    prev_err_[0] = 0.;
    prev_err_[1] = 0.;
    prev_out_[0] = 0.;
    prev_out_[1] = 0.;
    double prev_ril_err[3] = {0.,0.,0.};
}

//FC= filter coeff, cp tuh 
void PIDController::init(int mode, float KP, float KI, float KD, bool is_active)
{
    errorIntegral =0; 
    errorDerivative  = 0;
    err = 0;
    out = 0; 
    
    //INI PI 
    if (mode == 1) {
        Kp = KP;
        Ki = KI;
        Kd = 0;
    }


    // INI MAU PID 
    if(mode==2){  
        Kp = KP;
        Ki = KI;
        Kd = KD; 
    }

    is_active_ = is_active;
}



//UNTUK CLAMPING, GA DIUBAH
double PIDController::clamp(double val, double min, double max)
{
    if (val >= max)
    {
        return max;
    }
    else if (val <= min)
    {
        return min;
    }
    else
    {
        return val;
    }
}


double PIDController::compute_action(double target, double feedback, float samplingTime)
{
    err = target - feedback; //ITUNG ERROR
    targett = target;
    
    // Perhitungan I dan D
    errorIntegral = errorIntegral + err * samplingTime;
    errorDerivative = (prev_err_[0] - err)/samplingTime;
    
    errorDerivative = clamp(errorDerivative, -5,5);
    errorIntegral = clamp(errorIntegral, -500,500);
    
    if(is_active_)
    {
        // Basic PID / Rumus Biasa
        out += err * Kp + errorIntegral * Ki + errorDerivative * Kd ;

        //CLAMP OUTPUT 
        out = clamp(out, -1., 1.);
    }

    //ITUNG PREVIOUS ERROR 
    
    prev_err_[1] = prev_err_[0];
    prev_err_[0] = err;
    prev_out_[1] = prev_out_[0];
    prev_out_[0] = out;

    if(is_active_) {
        if (target == 0) {
            return 0.0;
        }
        else return clamp(out, -1., 1.); //-1 SAMA 1 TUH CLAMPING MIN MAX
        }
    else { 
        return 0.0;
    }
}


void PIDController::setActive(bool command_active) {
    is_active_ = command_active;
}