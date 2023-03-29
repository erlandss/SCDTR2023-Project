#ifndef DATAMODULE_H
#define DATAMODULE_H

#include "Arduino.h"
#include "Filter.h"

class DataModule
{
private:
    //Private member variables:
    uint8_t nodes[3];
    Filter coupling_gains_filter{0.60}; 
public:
    //Public member variables for easy access:
    float coupling_gains[3] = {0};
    //Public member functions:
    DataModule(){};
    ~DataModule(){};
    void init(uint8_t* nodes_);
    int at_node(uint8_t node);
    void update_gains(uint8_t node, int adc_val);
};

#endif