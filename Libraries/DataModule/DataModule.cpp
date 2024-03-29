#include "DataModule.h"

void DataModule::init(uint8_t* nodes_)
{
    for(int i = 0; i < 3; i++){
        nodes[i] = nodes_[i];
        costVector[i]=1;
    } 
}

int DataModule::at_node(uint8_t node){
    for(int i = 0; i < 2; i++){
        if(nodes[i] == node){
            return i;
        }
    }
    return 2;
}

void DataModule::update_gains(uint8_t node, int adc_val)
{
    //Convert to lux
    float v = adc_val*(3.3/4095);
    float ldr = 10000*(3.3-v)/v; //ldr = R*(Vcc-v)/v
    float lux = pow(10,-(log10(ldr)-6.1)/0.8);
    lux = coupling_gains_filter.filter_data(lux);
    coupling_gains[this->num_from_id(node)] = lux; 
}

void DataModule::update_lux_buffer(float lux_val){
    lux_buffer.add(lux_val);
}

void DataModule::update_pwm_buffer(int pwm_val){
    pwm_buffer.add(pwm_val);
}

void DataModule::update_metrics(unsigned long tk, float d, float L, float l){
    metrics.update(tk, d, L, l);
}

double DataModule::get_metric(char c){
    if(c == 'e'){
        return this->metrics.E;
    }
    else if(c == 'v'){
        return this->metrics.V;
    }
    else if(c == 'f'){
        return this->metrics.F;
    }
    return -1;
}

int DataModule::num_from_id(uint8_t id){
    int num = 3;
    for(int i = 0; i < 3; i++){
        if(id <= nodes[i]) num--;
    }
    return num;
}