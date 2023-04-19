#ifndef CONTROLMODULE_H
#define CONTROLMODULE_H

#include "Arduino.h"
#include "Pid.h"
#include "Filter.h"
#include "Queue.h"
class ControlModule
{
private:
    // Pid local_pid{0.02, 1.2, 8, 0.07, 1};
    Filter local_control_filter{0.90};
    Queue<float> receiveIntentQueue[2];
    Queue<float> sendIntentQueue;
    mutex_t sendQueueMtx;
    mutex_t receiveQueueMtx;
public:
    //Public member variables for easy access:
    Pid local_pid{0.02, 1.2, 8, 0.07, 1};
    int readyToReadQueue[2]={0};
    float lux_ref;
    float last_lux_read;
    int readyToSendQueue=0;
    //Public member functions:
    ControlModule(){
        mutex_init(&sendQueueMtx);
        mutex_init(&receiveQueueMtx);
    }
    ~ControlModule(){}
    void init_local_pid(float _h, float _K = 1, float _b = 1, float _Ti = 1, float _Tt = 1, float N_ = 10);
    int compute_local_control(int adc_val);
    void writeSendIntentQueue(float* _d);
    float* readSendIntentQueue();

    float* readReceiveIntentQueue(int index);
    void writeReceiveIntentQueue(float* _d,int index);
    //whichQueue==0 corresponds to emptying receiveQueue, other values means to empty sendQueue
    void emptyQueue(int whichQueue);

};

#endif