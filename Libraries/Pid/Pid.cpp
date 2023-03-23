#include "Pid.h"

Pid::Pid( float _h, float _K, float _b,
        float _Ti, float _Tt, float N_)
    // member variable initialization list
    : h {_h}, K {_K}, b {_b}, Ti {_Ti}, Tt {_Tt}, N {N_}, I {0.0}, y_old {0.0}
    { } // should check arguments validity

float Pid::compute_control( float r, float y ) {
    float P = K*(b*r-y);
    float v = P+I;
    float u = v;
    if( v < 0 ) u = 0;
    if( v > 4095 ) u = 4095;
    I += (h/Tt)*(u-v); //Back calculation for anti windup
    return u;
}