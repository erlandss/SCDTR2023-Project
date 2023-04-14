#ifndef DATAMODULE_H
#define DATAMODULE_H

#include "Arduino.h"
#include "Filter.h"
#include "CircularBuffer_.h"
#include "PerformanceMetrics.h"

class DataModule
{
private:
    //Private member variables:
    uint8_t nodes[3];
    Filter coupling_gains_filter{0.60};
public:
    //Public member variables for easy access:
    float coupling_gains[3] = {0};
    float external_disturbance = 0;
    CircularBuffer<float> lux_buffer{60*100};
    CircularBuffer<int> pwm_buffer{60*100};
    PerformanceMetrics metrics;
    float power_consumption = 0;
    //Public member functions:
    DataModule(){};
    ~DataModule(){};
    void init(uint8_t* nodes_);
    int at_node(uint8_t node);
    void update_gains(uint8_t node, int adc_val);
    void update_lux_buffer(float lux_val);
    void update_pwm_buffer(int pwm_val);
    void update_metrics(unsigned long tk, float d, float L, float l);
};

#endif