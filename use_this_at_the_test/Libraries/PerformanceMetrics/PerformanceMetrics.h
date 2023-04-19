#ifndef PerformanceMetrics_H
#define PerformanceMetrics_H
#include "Arduino.h"

class PerformanceMetrics{
private:
    unsigned long tk_;
    unsigned long N;
    float dk[3];
    int n;
public:
    double E;
    float V;
    float F;
    PerformanceMetrics() :
        tk_{0}, E{0}, V{0}, F{0}, N{0}, n{2} {
            dk[0], dk[1], dk[2] = 0;
        }
    void update(unsigned long tk, float d, float L, float l){
        dk[n-2] = dk[n-1];
        dk[n-1] = dk[n];
        dk[n] = d;
        N++;
        if(N == 1) return;

        E += 0.108*dk[n-1]*(tk-tk_);
        V += V/(N-N*N) + max(0, L-l)/N;
        float f = 0.0;
        if((dk[n]-dk[n-1])*(dk[n-1]-dk[n-2]) < 0.0){
            f = abs(dk[n]-dk[n-1]) + abs(dk[n-1]-dk[n-2]);
        }
        F += F/(N-N*N) + f/N;

        tk_ = tk;
    }
};

#endif