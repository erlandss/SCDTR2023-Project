#include "ControlModule.h"

void ControlModule::init_local_pid(float _h, float _K, float _b, float _Ti, float _Tt, float N_){
    Pid pid{_h, _K, _b, _Ti, _Tt, N_};
    this->local_pid = pid;
    this->lux_ref = 0;
}

int ControlModule::compute_local_control(int adc_val){
    //Convert to lux
    float v = adc_val*(3.3/4095);
    float ldr = 10000*(3.3-v)/v; //ldr = R*(Vcc-v)/v
    float lux = pow(10,-(log10(ldr)-6.1)/0.8);
    lux = (int)feedback_state*local_control_filter.filter_data(lux);
    last_lux_read = lux;
    float u = local_pid.compute_control(this->lux_ref, lux, (int)anti_windup_state);
    local_pid.housekeep(this->lux_ref, lux);
    return (int)u;
}