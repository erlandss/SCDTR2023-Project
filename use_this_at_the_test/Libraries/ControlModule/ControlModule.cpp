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
    last_lux_read = lux;
    lux = (int)feedback_state*local_control_filter.filter_data(lux);
    float u = local_pid.compute_control(this->lux_ref, lux, (int)anti_windup_state);
    local_pid.housekeep(this->lux_ref, lux);
    return (int)u;
}
void ControlModule::writeSendIntentQueue(float* _d){
    if(mutex_enter_timeout_ms(&sendQueueMtx,10)){
    for (int i =0; i<3; i++){
        sendIntentQueue.add(_d[i]);
    }
    readyToSendQueue=1;
    mutex_exit(&sendQueueMtx);
    }
}
float* ControlModule::readSendIntentQueue(){
    if(mutex_enter_timeout_ms(&sendQueueMtx,10)){

    static float intent[3];
    for (int i =0;i<3;i++){
        intent[i]=sendIntentQueue.pop();
    }
    readyToSendQueue=0;
    mutex_exit(&sendQueueMtx);
    return intent;
    }else{
        return nullptr;
    }
}
void ControlModule::writeReceiveIntentQueue(float* _d,int index){
    if(mutex_enter_timeout_ms(&receiveQueueMtx,10)){
    for (int i =0; i<3; i++){
        receiveIntentQueue[index].add(_d[i]);
    }
    readyToReadQueue[index]=1;
    mutex_exit(&receiveQueueMtx);
    }
}
float* ControlModule::readReceiveIntentQueue(int index){

    if(mutex_enter_timeout_ms(&receiveQueueMtx,10)){

    static float intent[3];
    for (int i =0;i<3;i++){
        intent[i]=receiveIntentQueue[index].pop();
    }
    readyToReadQueue[index]=0;
    mutex_exit(&receiveQueueMtx);
    return intent;
    }else{
        return nullptr;
    }
}

void ControlModule::emptyQueue(int whichQueue){

    if(whichQueue==0){
        if(mutex_enter_timeout_ms(&receiveQueueMtx,10)){
        for (int i=0; i<2;i++){
            while(!receiveIntentQueue[i].is_empty()){
                receiveIntentQueue[i].pop();
            }
            mutex_exit(&receiveQueueMtx);
        }

        }
    }else{
        if(mutex_enter_timeout_ms(&sendQueueMtx,10)){
        while(!sendIntentQueue.is_empty()){
            sendIntentQueue.pop();
        }
        mutex_exit(&sendQueueMtx);
        }        
    }
}
