#include "CanHelperModule.h"


void msg_to_bytes(uint32_t msg, uint8_t * bytes) {
  bytes[0] = msg;  bytes[1] = (msg >> 8);
  bytes[2] = (msg >> 16);  bytes[3] = (msg >> 24);
}

uint32_t can_frame_to_msg(can_frame * frm) {
  uint8_t b[4];
  b[0] = ICC_READ_DATA; b[1] = frm->can_id;
  b[2] = frm->data[0]; b[3] = frm->data[1];
  return bytes_to_msg(b);
}

uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg) {
  uint8_t b[4];
  b[3] = ICC_ERROR_DATA;  b[2] = 0;  
  b[1] = canintf;  b[0] = eflg;
  return bytes_to_msg(b);
}

uint32_t bytes_to_msg(uint8_t * b) {
  uint32_t b0 {b[0]}, b1 {b[1]},b2 {b[2]},b3 {b[3]}; 
  return b0 + (b1 << 8) + (b2 << 16) + (b3 << 24);
}

void print_message(int number, int node, int id, int val) {
  Serial.print(" number ");
  Serial.print( number );
  Serial.print(" at node " );
  Serial.print( node, HEX );
  Serial.print(" with id ");
  Serial.print( id, HEX );
  Serial.print(" : ");
  Serial.println( val );
}

void print_can_errors(uint8_t canintf, uint8_t eflg) {
  Serial.println( canintf_str );
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print("  "); 
    Serial.write( bitRead( canintf, bit ) ? '1' : '0' ); 
    Serial.print("   | "); 
  }
  Serial.println(".");
  Serial.println(eflg_str);
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print("  "); 
    Serial.write(bitRead(eflg, bit) ? '1' : '0'); 
    Serial.print("   | "); 
  } 
  Serial.println(".");
}

can_frame bytes_to_can_frame(uint8_t * bytes, int num_bytes, uint8_t id){
    can_frame frame;
    frame.can_id = id;
    frame.can_dlc = num_bytes;
    memcpy(frame.data, bytes, num_bytes);
    return frame;
}

int can_frame_to_bytes(can_frame * frame, uint8_t * bytes, uint8_t & id){
    int num_bytes = frame->can_dlc;
    id = frame->can_id;
    memcpy(bytes, frame->data, num_bytes);
    return num_bytes;
}

can_frame float_to_can_frame(float f, uint8_t id){
    uint8_t bytes[5];
    bytes[0] = CAN_WRITE_FLOAT;
    memcpy(&bytes[1], &f, 4);
    return bytes_to_can_frame(bytes, 5, id);
}
