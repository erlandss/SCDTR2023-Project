#ifndef SERIALHANDLER_H
#define SERIALHANDLER_H

#include "Arduino.h"
#include "Queue.h"
#include <mcp2515.h>

class SerialHandler
{
private:
    uint8_t nodes[3];
    Queue<char> print_queue;
    Queue<char> receieve_queue;
    mutex_t receieve_queue_mtx;
public:
    SerialHandler(){mutex_init(&receieve_queue_mtx);};
    ~SerialHandler(){};
    void init(uint8_t* nodes_);
    void read_inputs();
    void add_print(char c);
    void add_println(char c);
    void add_print(const char str[]);
    void add_println(const char str[]);
    void add_print(int num);
    void add_println(int num);
    void print_all();
    bool has_cmds();
    Queue<char> get_cmd();
};

#endif