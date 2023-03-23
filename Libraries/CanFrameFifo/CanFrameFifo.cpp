#include "CanFrameFifo.h"

inline int CanFrameFifo::put( can_frame *frame ) {
   if( write_lock ) return 0; //buffer full
   cf_buffer[ write_index ] = *frame;
   write_index = ( ++write_index ) % buffsize;
   if( write_index == read_index)  write_lock = true;
   return 1;
}

inline int CanFrameFifo::get( can_frame *frame ) {
   if( !write_lock  && ( read_index == write_index ) ) 
      return 0; //empty buffer
   if( write_lock && ( read_index == write_index ) ) 
      write_lock = false; //release lock
  *frame = cf_buffer[ read_index ];
   read_index = ( ++read_index ) % buffsize;
   return 1;
}
