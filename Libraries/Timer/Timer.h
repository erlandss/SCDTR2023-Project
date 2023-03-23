#ifndef Timer_H
#define Timer_H
#include "Arduino.h"

class Timer{
private:
 unsigned long time_to_excecute_;
 unsigned long time_delay_;

public:
    Timer(unsigned long time_delay) :
        time_delay_{time_delay}, time_to_excecute_{0} {};

    void reset(){
        time_to_excecute_ = millis() + time_delay_;
    }

    bool timed_out(){
        return (millis() >= time_to_excecute_);
    }
};

#endif