#include <SPI.h>
#include <hardware/flash.h>
#include <mcp2515.h>
#include "Timer.h"
#include "DataModule.h"
#include "ControlModule.h"
#include "CanHelperModule.h"

const int DAC_RANGE = 4096;

//Pins:
const int LED_PIN = 15;
const int LDR_PIN = A0;

//Modules:
DataModule data;
ControlModule control;

//Timers for interrupts:
struct repeating_timer local_control_timer;
//struct repeating_timer core_comm_timer;

//Can stuff:
MCP2515 can0 {spi0, 17, 19, 16, 18, 10000000};
const uint8_t interruptPin {20};
volatile bool got_irq {false};
void read_interrupt(uint gpio, uint32_t events) {
   got_irq = true;
} 

//Node stuff:
enum node_state {
  NODE_AWAKENING = 1,
  NODE_CALIBRATING = 2,
  NODE_IDLE = 3,
  NODE_DISCUSSING = 4
};
int node_state = NODE_AWAKENING;
bool first_time_idle = true; 
uint8_t nodes[3] = {0, 0, 0};

//Setup for core0:
void setup() {
  uint8_t pico_flash_id[8];
  rp2040.idleOtherCore(); 
  //flash calls are unsafe if two cores are operating
  flash_get_unique_id(pico_flash_id); 
  rp2040.resumeOtherCore();
  nodes[0] = pico_flash_id[7];
  Serial.begin(115200);

  analogReadResolution(12); //default is 10
  analogWriteFreq(60000); //60KHz, about max
  analogWriteRange(DAC_RANGE); //100% duty cycle

  //add_repeating_timer_us( -100, core_comm_timer_callback, NULL, &core_comm_timer);  

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(1000); 
}

//Setup for core1:
void setup1() {
  can0.reset();
  can0.setBitrate(CAN_1000KBPS); 
  can0.setNormalMode();
  gpio_set_irq_enabled_with_callback( interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt );
  delay(1000);
}

//State machine on core0:
void loop() {
  switch(node_state){
    case NODE_AWAKENING : {
      //Some "random" delay to make sure all nodes dont send at exactly the same time, overfilling the can bus
      int wait = (nodes[0]*17)%101;
      Timer write_awake_timer(100+wait);
      write_awake_timer.reset();
      while(1){
        uint32_t msg;
        uint8_t b[4];

        if(write_awake_timer.timed_out()){
          b[0] = ICC_WRITE_DATA;
          b[1] = nodes[0];
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
              delay(100+wait);
              b[0] = ICC_WRITE_DATA;
              b[1] = nodes[0];
              b[2] = CAN_AWAKE; 
              b[3] = 0;
              rp2040.fifo.push(bytes_to_msg(b));                
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
      //Give calibration turn based on unique id
      data.init(&nodes[0]);
      int turn = 2;
      if(nodes[0] < nodes[1] || nodes[0] < nodes[2]){
        turn--;
        if(nodes[0] < nodes[1] && nodes[0] < nodes[2]){
          turn--;
        }
      }
      //To make sure all nodes have left init state
      Timer wait_for_all_nodes(250);
      wait_for_all_nodes.reset();

      while(1){
        uint32_t msg;
        uint8_t b[4];
        if(!turn && wait_for_all_nodes.timed_out() && data.coupling_gains[0] == 0){
          b[0] = ICC_WRITE_DATA;
          b[1] = nodes[0];
          b[2] = CAN_CALIBRATING; 
          b[3] = 0;
          rp2040.fifo.push(bytes_to_msg(b));
          analogWrite(LED_PIN, DAC_RANGE-1);
          delay(250);
          for(int i = 0; i < 10; i++){
              //Use the filtered value over 10 measurements
              int read_adc = analogRead(LDR_PIN);
              data.update_gains(nodes[0], read_adc);
              delay(10);
          }
          delay(500);
          analogWrite(LED_PIN, 0);
        }
        
        if( rp2040.fifo.pop_nb(& msg) ) {
          msg_to_bytes(msg, b);
          if(b[0] == ICC_READ_DATA && b[2] == CAN_CALIBRATING) {
            uint8_t node = b[1];
            delay(250);
            for(int i = 0; i < 10; i++){
              //Use the filtered value over 10 measurements
              int read_adc = analogRead(LDR_PIN);
              data.update_gains(node, read_adc);
              delay(10);
            }
            turn--;
            delay(1250);
          }
        }

        if(data.coupling_gains[0] != 0 && data.coupling_gains[1] != 0 && data.coupling_gains[2] != 0){
          node_state = NODE_IDLE;
          Serial.print("Done calibrating with the following coupling gains: ");
          Serial.print(data.coupling_gains[0]); Serial.print(", ");
          Serial.print(data.coupling_gains[1]); Serial.print(", ");
          Serial.println(data.coupling_gains[2]);
          break;
        }
      }
    }

    case NODE_IDLE : {
      if(first_time_idle){
        control.init_local_pid(0.02, 1.2, 8, 0.07, 1); //h, K, b, Ti, Tt
        add_repeating_timer_ms( -10, local_control_timer_callback, NULL, &local_control_timer);
        first_time_idle = false;

      }
      
      while(1){
        handleSerial();
        delayMicroseconds(10);
      }

    }

    case NODE_DISCUSSING : {
      
    }

    default : 
      break;
  }
}

//Callbacks for core0:
bool local_control_timer_callback( struct repeating_timer *t ){
  int read_adc = analogRead(LDR_PIN);
  int u = control.compute_local_control(read_adc);
  analogWrite(LED_PIN, u);
  data.update_lux_buffer(control.last_lux_read);
  data.update_pwm_buffer(u);
  data.update_metrics(millis(), u/4095, control.lux_ref, control.last_lux_read);
  return true;
}

//State machine on core1:
void loop1() {
  switch(node_state){
    case NODE_AWAKENING : { 
      while(1){
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
        //read fifo and write can-bus
        if( rp2040.fifo.pop_nb( &msg ) ) {
          msg_to_bytes( msg , b );
          if( b[0] == ICC_WRITE_DATA ) {
            frm.can_id = b[1]; 
            frm.can_dlc = 2; 
            frm.data[0] = b[2]; 
            frm.data[1] = b[3];
            can0.sendMessage(&frm);
          }
        }

        if(node_state != NODE_AWAKENING) break;
      }
    }

    case NODE_CALIBRATING : {
      while(1){
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
        //read fifo and write can-bus
        if( rp2040.fifo.pop_nb( &msg ) ) {
          msg_to_bytes( msg , b );
          if( b[0] == ICC_WRITE_DATA ) {
            frm.can_id = b[1]; 
            frm.can_dlc = 2; 
            frm.data[0] = b[2]; 
            frm.data[1] = b[3];
            can0.sendMessage(&frm);
          }
        }

        if(node_state != NODE_CALIBRATING) break;
      }
    }

    case NODE_IDLE : {
      while(1){
        can_frame frm;
        if(got_irq) { 
          got_irq = false;
          uint8_t irq = can0.getInterrupts();
          if(irq & MCP2515::CANINTF_RX0IF) {
            can0.readMessage( MCP2515::RXB0, &frm ); 
          }
          if(irq & MCP2515::CANINTF_RX1IF) {
            can0.readMessage(MCP2515::RXB1, &frm);
          }
          if( can0.checkError()) {
            uint8_t err = can0.getErrorFlags();
            //Do something with this annoying error
          }
          switch(frm.data[0]) {
            case CAN_WRITE_FLOAT : {
              float f;
              memcpy(&f, &frm.data[1], 4);
              //Do stuff with float
              break;
            }
            default:
              break;
          }
        }        
      }
    }

    case NODE_DISCUSSING : {
      while(1){
        can_frame frm;
        if(got_irq) { 
          got_irq = false;
          uint8_t irq = can0.getInterrupts();
          if(irq & MCP2515::CANINTF_RX0IF) {
            can0.readMessage( MCP2515::RXB0, &frm ); 
          }
          if(irq & MCP2515::CANINTF_RX1IF) {
            can0.readMessage(MCP2515::RXB1, &frm);
          }
          if( can0.checkError()) {
            uint8_t err = can0.getErrorFlags();
            //Do something with this annoying error
          }
          switch(frm.data[0]) {
            case CAN_WRITE_FLOAT : {
              float f;
              memcpy(&f, &frm.data[1], 4);
              //Do stuff with float
              break;
            }
            default:
              break;
          }
        }
      }
    }

    default:
      break;
  }
}


void handleSerial(){
  if(Serial.available()){
    char cmd = Serial.read();
    float val;
    if(Serial.available()){
      val = Serial.parseInt();
    }
    switch(cmd){
      case 'r':
        //set lux reference in interval 0 to 100
        control.lux_ref = val;
        break;
      default:
        break;     
    }
  }
}
