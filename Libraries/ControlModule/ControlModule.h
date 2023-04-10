#ifndef CONTROLMODULE_H
#define CONTROLMODULE_H

#include "Arduino.h"
#include "Pid.h"
#include "Filter.h"

class ControlModule
{
private:
    Pid local_pid{0.02, 1.2, 8, 0.07, 1};
    Filter local_control_filter{0.90};
public:
    //Public member variables for easy access:
    float lux_ref;
    float last_lux_read;
    //Public member functions:
    ControlModule(){}
    ~ControlModule(){}
    void init_local_pid(float _h, float _K = 1, float _b = 1, float _Ti = 1, float _Tt = 1, float N_ = 10);
    int compute_local_control(int adc_val);
};

#endif