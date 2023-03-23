#include <SPI.h>
#include <hardware/flash.h>
#include <mcp2515.h>
#include "Timer.h"

//MCP2515 can0 {spi0};
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000}; 

//core1 responsible for can bus communication, core0 for most else
enum inter_core_cmds { 
  //From core1 to core0: contains data read (16 bit)
  ICC_READ_DATA = 1,  
  // From core0 to core1: contains data to write (16 bit)
  ICC_WRITE_DATA = 2, 
  // From core1 to core0: contains regs CANINTF, EFLG
  ICC_ERROR_DATA = 3
};

enum can_cmds {
  //Wake up
  CAN_AWAKE = 1,
  //
  CAN_AWAKE_ACK = 2
};

enum node_state {
  NODE_AWAKENING = 1,
  NODE_CALIBRATING = 2,
  NODE_OPERATING = 3
};

int node_state = NODE_AWAKENING; 
uint8_t node_address;
uint8_t nodes[3] = {0, 0, 0};


void setup() {
  uint8_t pico_flash_id[8];
  rp2040.idleOtherCore(); 
  //flash calls are unsafe if two cores are operating
  flash_get_unique_id(pico_flash_id); 
  rp2040.resumeOtherCore();
  node_address = pico_flash_id[7];
  nodes[0] = node_address;
  Serial.begin(115200);
  delay(1000);   

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

const uint8_t interruptPin {20};
volatile bool got_irq {false};
void read_interrupt(uint gpio, uint32_t events) {
   got_irq = true;
}

void setup1() {
  can0.reset();
  can0.setBitrate(CAN_1000KBPS); 
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback( interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt );
  delay(1000);
}

void loop() {
  switch(node_state){
    case NODE_AWAKENING : {
      Timer write_awake_timer(100);
      write_awake_timer.reset();
      while(1){
        uint32_t msg;
        uint8_t b[4];

        if(write_awake_timer.timed_out()){
          b[0] = ICC_WRITE_DATA;
          b[1] = node_address;
          b[2] = CAN_AWAKE; 
          b[3] = 0;
          rp2040.fifo.push(bytes_to_msg(b));
          write_awake_timer.reset();
        }
        
        if( rp2040.fifo.pop_nb(& msg) ) {
          msg_to_bytes(msg, b);
          if(b[0] == ICC_READ_DATA && b[2] == CAN_AWAKE) {
            if(nodes[1] == 0){
              nodes[1] = b[1];
            }
            else if(nodes[2] == 0 && b[1] != nodes[1]){
              nodes[2] = b[1];
              //To be sure the other nodes get node_adress of this node
              for(int i = 0; i < 100; i++){
                b[0] = ICC_WRITE_DATA;
                b[1] = node_address;
                b[2] = CAN_AWAKE; 
                b[3] = 0;
                rp2040.fifo.push(bytes_to_msg(b));
                delay(100);                
              }
              Serial.print("Node: "); Serial.print(nodes[0]); 
              Serial.print(" done waking up. In connection with other nodes: ");
              Serial.print(nodes[1]); Serial.print(" and "); Serial.println(nodes[2]);
              node_state = NODE_CALIBRATING;
              break;
            }
          }
          else if(b[0] == ICC_ERROR_DATA) {
            print_can_errors(b[2],b[3]);
          }
        }
      }      
      break;
    }
    case NODE_CALIBRATING : {
      break;
    }
    case NODE_OPERATING : {

    }
    default : 
      break;
  }
}

void loop1() {
  can_frame frm;
  uint32_t msg;
  uint8_t b[4];
  //reading the can-bus and writing the fifo
  if(got_irq) { 
    got_irq = false;
    uint8_t irq = can0.getInterrupts();
    if(irq & MCP2515::CANINTF_RX0IF) {
      can0.readMessage( MCP2515::RXB0, &frm );
      rp2040.fifo.push_nb(can_frame_to_msg( &frm ) ); 
    }
    if(irq & MCP2515::CANINTF_RX1IF) {
      can0.readMessage(MCP2515::RXB1, &frm);
      rp2040.fifo.push_nb(can_frame_to_msg( &frm ) ); 
    }
    if( can0.checkError()) {
      uint8_t err = can0.getErrorFlags();
      rp2040.fifo.push_nb(error_flags_to_msg(irq, err));
    }
  }
  if( rp2040.fifo.pop_nb( &msg ) ) {//read fifo write bus
    msg_to_bytes( msg , b );
    if( b[0] == ICC_WRITE_DATA ) {
      frm.can_id = b[1]; 
      frm.can_dlc = 2; 
      frm.data[0] = b[2]; 
      frm.data[1] = b[3];
      can0.sendMessage(&frm);
    }
  }
}

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

char canintf_str[] {"MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | "};
char eflg_str []   {"RX1OV | RX0OV | TXBO  | TXEP  | RXEP  | TXWAR | RXWAR | EWARN | "};

void print_message(int number, int node, int id, int val){
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
