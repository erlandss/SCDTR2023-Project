#ifndef CANHELPERMODULE_H
#define CANHELPERMODULE_H

#include "Arduino.h"
#include <mcp2515.h>

const char canintf_str[65] {"MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | "};
const char eflg_str [65]   {"RX1OV | RX0OV | TXBO  | TXEP  | RXEP  | TXWAR | RXWAR | EWARN | "};

enum can_cmds {
  //Wake up
  CAN_AWAKE = 1,
  //
  CAN_CALIBRATING = 2,
  //
  CAN_WRITE_FLOAT = 3  
};

//Communication between cores (core1 responsible for can bus communication, core0 for most else):
enum inter_core_cmds { 
  //From core1 to core0: contains data read (16 bit)
  ICC_READ_DATA = 1,  
  // From core0 to core1: contains data to write (16 bit)
  ICC_WRITE_DATA = 2, 
  // From core1 to core0: contains regs CANINTF, EFLG
  ICC_ERROR_DATA = 3
};

//Helper functions:
void msg_to_bytes(uint32_t msg, uint8_t * bytes);
uint32_t can_frame_to_msg(can_frame * frm);
uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg);
uint32_t bytes_to_msg(uint8_t * b);
void print_message(int number, int node, int id, int val);
void print_can_errors(uint8_t canintf, uint8_t eflg);
can_frame bytes_to_can_frame(uint8_t * bytes, int num_bytes, uint8_t id);
int can_frame_to_bytes(can_frame * frame, uint8_t * bytes, uint8_t & id); //Write the can message to the given variables and return the num of bytes in the message
can_frame float_to_can_frame(float f, uint8_t id);

#endif