#ifndef CANFRAMEFIFO_H
#define CANFRAMEFIFO_H

#include <mcp2515.h>

class CanFrameFifo {
  static const int buffsize = 10; //increase if needed
  can_frame cf_buffer[ buffsize ]; 
  int read_index;  //where to read next message
  int write_index; //where to write next message
  bool write_lock; //buffer full
public: 
   CanFrameFifo() : //constructor
      read_index{0}, write_index{0},write_lock{false} {};
   int put( can_frame * );
   int get( can_frame * );
};


#endif