#include "SerialHandler.h"

void SerialHandler::init(uint8_t* nodes_)
{
    for(int i = 0; i < 3; i++){
        nodes[i] = nodes_[i];
    } 
}

void SerialHandler::read_inputs(){    
    if(Serial.available()){
        if(mutex_enter_timeout_ms(&receieve_queue_mtx, 10)){
            do{
                receieve_queue.add(Serial.read());
            }
            while(Serial.available());
            receieve_queue.add('\0');
            mutex_exit(&receieve_queue_mtx);
        }
    }    
}

void SerialHandler::add_print(char c){
    print_queue.add(c);
}

void SerialHandler::add_println(char c){
    add_print(c);
    print_queue.add('\n');
}

void SerialHandler::add_print(const char str[]){
    for(int i = 0; str[i] != '\0'; i++){
        print_queue.add(str[i]);
    }
}

void SerialHandler::add_println(const char str[]){
    add_print(str);
    print_queue.add('\n');
}

void SerialHandler::add_print(int num){
    if (num == 0) {
        print_queue.add('0');
        return;
    }
    int numDigits = 0, divisor = 1, temp = num;
    while(temp){ 
        numDigits++; 
        divisor *= 10;
        temp /= 10; 
    }
    divisor /= 10;
    for (int i = 0; i < numDigits; i++) {
        int digit = (num / divisor) % 10;
        print_queue.add(digit + '0');
        divisor /= 10;
    }
}

void SerialHandler::add_println(int num){
    add_print(num);
    print_queue.add('\n');
}

void SerialHandler::add_print(float num){
    if (num == 0.0) {
        add_print("0.0");
        return;
    }
    int intPart = (int) num;
    add_print(intPart);
    float floatPart = num - intPart;
    if (floatPart > 0.0) {
        print_queue.add('.');
        int decimalDigits = 0;
        while (decimalDigits < 2 && floatPart > 0.0) {
            floatPart *= 10;
            int digit = (int) floatPart;
            print_queue.add(digit + '0');
            floatPart -= digit;
            decimalDigits++;
        }
    }
}

void SerialHandler::add_println(float num){
    add_print(num);
    print_queue.add('\n');
}

void SerialHandler::add_print(uint8_t b[], int num_bytes){
    for(int i = 0; i < num_bytes; i++){
        add_print((char)b[i]);
    }
}

void SerialHandler::print_all(){
    if(!print_queue.is_empty()){
        Serial.print(print_queue.pop());
    }
}

bool SerialHandler::has_cmds(){
    return !receieve_queue.is_empty();
}

Queue<char> SerialHandler::get_cmd(){
    Queue<char> cmd;
    if(mutex_enter_timeout_ms(&receieve_queue_mtx, 1)){
        char c = receieve_queue.pop();
        while(c != '\0'){
            cmd.add(c);
            c = receieve_queue.pop();
        }
        mutex_exit(&receieve_queue_mtx);
    }
    return cmd;
}