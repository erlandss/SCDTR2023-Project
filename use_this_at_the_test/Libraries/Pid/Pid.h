#ifndef PID_H
#define PID_H

class Pid {
    float I, K, Ti, Tt, b, h, y_old, N;
public:
    explicit Pid( float _h, float _K = 1, float _b = 1, float _Ti = 1, float _Tt = 1, float N_ = 10);
    ~Pid() {};
    float compute_control( float r, float y, int anti_windup_state);
    void housekeep( float r, float y);
    void set_B(float _b);
    float get_B();
};

inline void Pid::housekeep( float r, float y ) {
    float e = r - y;
    I += K*h/Ti*e;
    y_old = y;
}
inline void Pid::set_B(float _b){
    b=_b;
}
inline float Pid::get_B(){
    return b;
}

#endif
