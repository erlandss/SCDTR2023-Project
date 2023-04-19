#include <SPI.h>
#include <hardware/flash.h>
#include <mcp2515.h>
#include "Timer.h"
#include "DataModule.h"
#include "ControlModule.h"
#include "CanHelperModule.h"
#include "SerialHandler.h"
#include "Queue.h"
#include "Optimize2.h"
const int DAC_RANGE = 4096;

//Pins:
const int LED_PIN = 15;
const int LDR_PIN = A0;

//Modules:
DataModule data;
ControlModule control;
SerialHandler serial;

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

//Bools for controlling streaming and what not
bool stream_l_local = false;
bool stream_d_local = false;
bool stream_l_can = false;
bool stream_d_can = false;

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
      serial.init(&nodes[0]);
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
        if(!turn && wait_for_all_nodes.timed_out() && data.coupling_gains[data.num_from_id(nodes[0])] == 0){
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
      Serial.print("c0 idle");

      if(first_time_idle){
        control.init_local_pid(0.02, 1.2, 8, 0.07, 1); //h, K, b, Ti, Tt
        add_repeating_timer_ms( -10, local_control_timer_callback, NULL, &local_control_timer);
        first_time_idle = false;
      }
      while(1){
        serial.read_inputs();
        serial.print_all();
        if(node_state!=NODE_IDLE){
          break;          
        }
      }
    }

    case NODE_DISCUSSING : {

      float* intent2;
      float* intent3;
      //Change second argument to external disturbance
      Optimizer opt{0.07,10,control.lux_ref,data.coupling_gains,data.costVector, data.num_from_id(nodes[0])};

      opt.initOptimizer();
      opt.solveQP();

      delay(1000);
      for(int j=0;j<20;j++){
          // Serial.print("kuk\n");
          control.writeSendIntentQueue(opt.get_d());
          delay(1);
          
          while(control.readyToReadQueue[0]==0 || control.readyToReadQueue[1]==0){
            serial.read_inputs();
            serial.print_all();
          }

          Serial.print("iterating consensus ");
          intent2=control.readReceiveIntentQueue(0);
          intent3=control.readReceiveIntentQueue(1);

          opt.update_dbar(opt.get_d(),intent2,intent3);
          opt.update_y();
          
          opt.solveQP();
          Serial.print(j);
          Serial.print("\n");
        // } 
        

      }

    if(opt.get_d()[data.num_from_id(nodes[0])]>=0){
      control.local_pid.set_B(opt.get_d()[data.num_from_id(nodes[0])]/control.lux_ref);
    }
      
      Serial.print("B:");
      Serial.print(control.local_pid.get_B());
      node_state=NODE_IDLE;
      // Serial.print("Going idle\n");
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
  if(stream_l_local){
    serial.add_print("s l 0 "); serial.add_print(control.last_lux_read);
    serial.add_print(' '); serial.add_println((int)millis());
  }
  if(stream_d_local){
    serial.add_print("s d 0 "); serial.add_print(u);
    serial.add_print(' '); serial.add_println((int)millis());
  }

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
      Serial.print("c1 idle\n");
      bool should_break = false;
      while(1){
        if(node_state!=NODE_IDLE){
          break;
        }
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
            break;
          }

          if(frm.data[1] == nodes[0]){
            switch(frm.data[0]) {
              case CAN_SET_ILLUMINANCE_REF : {
                // serial.add_print("got new ref\n");
                if(frm.data[2]==nodes[0]){
                  float f;
                  memcpy(&f, &frm.data[3], 4);
                  control.lux_ref = f;
                  // break;
                }
                node_state=NODE_DISCUSSING;
                should_break=true;
                break;
              }
              case CAN_GET_ILLUMINANCE_REF : {
                uint8_t b[6];
                b[0] = CAN_REPLY_ILLUMINANCE_REF;
                b[1] = frm.can_id;
                memcpy(&b[2], &control.lux_ref, 4);
                can_frame frm = bytes_to_can_frame(b, 6, nodes[0]);
                can0.sendMessage(&frm);
                break;
              }
              case CAN_REPLY_ILLUMINANCE_REF : {
                float f;
                memcpy(&f, &frm.data[2], 4);
                serial.add_print("r ");
                serial.add_print(data.at_node(frm.can_id)); serial.add_print(' ');
                serial.add_println(f);
                break;
              }
              case CAN_GET_MINUTE_BUFFER_L : {
                int reply_id = frm.can_id; 
                uint8_t b[7];
                b[0] = CAN_ADD_BYTES_TO_PRINT;
                b[1] = reply_id;
                b[2] = 'b';
                b[3] = ' ';
                b[4] = 'l';
                b[5] = ' ';
                b[6] = nodes[0];
                can_frame frm = bytes_to_can_frame(b, 7, nodes[0]);
                can0.sendMessage(&frm);
                const float* p_lux = data.lux_buffer.get_buffer();
                for(int i = 0; i < data.lux_buffer.size_; i++){
                  float val = p_lux[(data.lux_buffer.head_ + i) % data.lux_buffer.capacity_];
                  uint8_t b[6];
                  b[0] = CAN_REPLY_MINUTE_BUFFER_L;
                  b[1] = reply_id;
                  memcpy(&b[2], &val, 4);
                  can_frame frm = bytes_to_can_frame(b, 6, nodes[0]);
                  delayMicroseconds(100);
                  can0.sendMessage(&frm);
                }
                break;
              }
              case CAN_GET_MINUTE_BUFFER_D : {
                int reply_id = frm.can_id; 
                uint8_t b[7];
                b[0] = CAN_ADD_BYTES_TO_PRINT;
                b[1] = reply_id;
                b[2] = 'b';
                b[3] = ' ';
                b[4] = 'd';
                b[5] = ' ';
                b[6] = nodes[0];
                can_frame frm = bytes_to_can_frame(b, 7, nodes[0]);
                can0.sendMessage(&frm);
                const int* p_pwm = data.pwm_buffer.get_buffer();
                for(int i = 0; i < data.pwm_buffer.size_; i++){
                  int val = p_pwm[(data.pwm_buffer.head_ + i) % data.pwm_buffer.capacity_];
                  uint8_t b[6];
                  b[0] = CAN_REPLY_MINUTE_BUFFER_L;
                  b[1] = reply_id;
                  memcpy(&b[2], &val, 4);
                  can_frame frm = bytes_to_can_frame(b, 6, nodes[0]);
                  delayMicroseconds(100);
                  can0.sendMessage(&frm);
                }
              }
              case CAN_REPLY_MINUTE_BUFFER_L : {
                float f;
                memcpy(&f, &frm.data[2], 4);
                serial.add_print(' ');
                serial.add_print(f);
                break;
              }
              case CAN_REPLY_MINUTE_BUFFER_D : {
                int i;
                memcpy(&i, &frm.data[2], 4);
                serial.add_print(' ');
                serial.add_print(i);
                break;
              }
              case CAN_START_STREAM_L : {
                stream_l_can = true;
                break;
              }
              case CAN_START_STREAM_D : {
                stream_d_can = true;
                break;
              }
              case CAN_STOP_STREAM_L : {
                stream_l_can = false;
                break;
              }
              case CAN_STOP_STREAM_D : {
                stream_d_can = false;
                break;
              }
              case CAN_REPLY_STREAM_L : {
                float f;
                memcpy(&f, &frm.data[2], 4);
                serial.add_print("s l "); serial.add_print(data.at_node(frm.can_id)); 
                serial.add_print(' '); serial.add_print(f);
                serial.add_print(' '); serial.add_println((int)millis());
                break;
              }
              case CAN_REPLY_STREAM_D : {
                int i;
                memcpy(&i, &frm.data[2], 4);
                serial.add_print("s d "); serial.add_print(data.at_node(frm.can_id)); 
                serial.add_print(' '); serial.add_print(i);
                serial.add_print(' '); serial.add_println((int)millis());
                break;
              }
              case CAN_ADD_BYTES_TO_PRINT : {
                int num_bytes = frm.can_dlc-2;
                uint8_t b[num_bytes];
                memcpy(&b[0], &frm.data[2], num_bytes);
                serial.add_print(&b[0], num_bytes);
                break;
              }
              default:
                break;
            }
          }
        }

        handle_inputs();
        if(should_break)break;        
      }
    }

    case NODE_DISCUSSING : {
      Serial.print("core 1 discussing\n");
      float receiveIntentBuffer1[3];
      float receiveIntentBuffer2[3];
      float* intentVector;
      int sending=0;
      int sendNum=0;
      unsigned long previousTime=0;
      unsigned long currentTime=0;
      // delay(1);
      while(1){
        // Serial.print("kjor\n");
        can_frame frm;
        currentTime =millis();
        
        if(control.readyToSendQueue){
          // Serial.print("intent ready\n");
          intentVector=control.readSendIntentQueue();
          sending=1;          
        }
          // for(int i=0;i<3; i++){
            if(sending && (currentTime-previousTime)>10){
            uint8_t b[5];
            switch (sendNum)
            {
            case 0:
              b[0] = CAN_SEND_INTENT1;
              break;
            case 1:
              b[0] = CAN_SEND_INTENT2;
              break;
            case 2:
              b[0] = CAN_SEND_INTENT3;
              sending=0;
              break;
            
            default:
              break;
            }  
                
                // Serial.print(sendNum);
                memcpy(&b[1], &(intentVector[sendNum]), 4);
                frm = bytes_to_can_frame(b, 5, nodes[0]);
                can0.sendMessage(&frm);
                sendNum++;
                if(sendNum>2){
                  sendNum=0;
                }
                // delayMicroseconds(200);  
                previousTime=currentTime;   
                Serial.print("Sent intent\n");       
            }

            // Serial.print("Sending intent\n");

          // }
          
        


        if(got_irq) { 


          got_irq = false;
          uint8_t irq = can0.getInterrupts();
          if(irq & MCP2515::CANINTF_RX0IF) {
            can0.readMessage( MCP2515::RXB0, &frm ); 
            // rp2040.fifo.push_nb(can_frame_to_msg( &frm ) ); 
          }
          if(irq & MCP2515::CANINTF_RX1IF) {
            can0.readMessage(MCP2515::RXB1, &frm);
            // rp2040.fifo.push_nb(can_frame_to_msg( &frm ) ); 
          }
          if( can0.checkError()) {
            
            uint8_t err = can0.getErrorFlags();
            print_can_errors(irq,err);
            can0.clearRXnOVRFlags();
            // can0.clearInterrupts();
            continue;
            //Do something with this annoying error
            
          }
          else{
            if(frm.can_id==nodes[1]){
              Serial.print("Got irq from node 1");
              // Serial.print(frm.data[0]); 
              // Serial.print("\n"); 

              switch (frm.data[0])
              {
              case CAN_SEND_INTENT1:
                memcpy(&receiveIntentBuffer1[0], &frm.data[1], 4);
                break;
              case CAN_SEND_INTENT2:
                memcpy(&receiveIntentBuffer1[1], &frm.data[1], 4);
                break;
              case CAN_SEND_INTENT3:
                memcpy(&receiveIntentBuffer1[2], &frm.data[1], 4);
                control.writeReceiveIntentQueue(receiveIntentBuffer1,0);
                // Serial.print(control.readyToReadQueue[0]);
                // Serial.print(control.readyToReadQueue[1]);
                // Serial.print("\n");                
                Serial.print("received intent\n");
                break;              
              default:
                break;
              }

            }else if(frm.can_id==nodes[2]){
              Serial.print("Got irq from node 2");  
              // Serial.print(frm.data[0]); 
              // Serial.print("\n");          
              switch (frm.data[0])
              {
              case CAN_SEND_INTENT1:
                memcpy(&receiveIntentBuffer2[0], &frm.data[1], 4);
                break;
              case CAN_SEND_INTENT2:
                memcpy(&receiveIntentBuffer2[1], &frm.data[1], 4);
                break;
              case CAN_SEND_INTENT3:
                memcpy(&receiveIntentBuffer2[2], &frm.data[1], 4);
                Serial.print("received intent\n");

                control.writeReceiveIntentQueue(receiveIntentBuffer2,1);
                // Serial.print(control.readyToReadQueue[0]);
                // Serial.print(control.readyToReadQueue[1]);
                // Serial.print("\n");
                break;              
              default:
                break;
              }
            }
            // Serial.print("kuse\n");
            // Serial.print(frm.data[0]);
            
          }
        }
        
      if(node_state!=NODE_DISCUSSING){
        // Serial.print("Gar ut\n");        
        break;        
      }   
      }
    }

    default:
      break;
  }
}

void handle_inputs(){
  if(serial.has_cmds()){
    Queue<char> cmd = serial.get_cmd();
    char c1 = cmd.pop();
    switch(c1){
      case 'g' : {
        //All getter commands
        char c2 = cmd.pop();
        uint8_t i = cmd.pop() - '0'; //Should be 0, 1 or 2, where 0 is this node
        switch(c2){
          case 'd': {
            //Get current duty cycle at luminaire i
            break;
          }
          case 'r': {
            //Get current illuminance reference at luminaire i
            if(!i){
              serial.add_print("r ");
              serial.add_print(i); serial.add_print(' ');                 
              serial.add_println(control.lux_ref);
            }
            else{
              uint8_t b[2];
              b[0] = CAN_GET_ILLUMINANCE_REF;
              b[1] = nodes[i];
              can_frame frm = bytes_to_can_frame(b, 2, nodes[0]);
              can0.sendMessage(&frm);
            }
            break;
          }
          case 'l': {
            //Get measured illuminance at luminaire i
            break;
          }
          case 'o': {
            //Get current occupancy state at desk <i>
            break;
          }
          case 'a': {
            //Get anti-windup state at desk <i>
            break;
          }
          case 'k': {
            //Get feedback state at desk <i>
            break;
          }
          case 'x': {
            //Get current external illuminance at desk <i>
            break;
          }
          case 'p': {
            //Get instantaneous power consumption at desk <i>
            break;
          }
          case 't': {
            //Get elapsed time since last restart
            break;
          }
          case 'b': {
            char x = cmd.pop();
            //Get last minute buffer of variable <x> of desk <i>. <x> can be “l” or “d”.
            if(!i){
              serial.add_print("b "); serial.add_print(x); serial.add_print(' '); serial.add_print(i);                
              if(x == 'l'){
                const float* p_lux = data.lux_buffer.get_buffer();
                for(int i = 0; i < data.lux_buffer.size_; i++){
                  serial.add_print(' ');
                  float val = p_lux[(data.lux_buffer.head_ + i) % data.lux_buffer.capacity_];
                  serial.add_print(val);
                }
              }
              else if(x == 'd'){
                const int* p_pwm = data.pwm_buffer.get_buffer();
                for(int i = 0; i < data.pwm_buffer.size_; i++){
                  serial.add_print(' ');
                  int val = p_pwm[(data.pwm_buffer.head_ + i) % data.pwm_buffer.capacity_];
                  serial.add_print(val);
                }
              }
              serial.add_print('\n');
            }
            else{
              uint8_t b[2];
              b[0] = (x == 'l') ? CAN_GET_MINUTE_BUFFER_L : CAN_GET_MINUTE_BUFFER_D;
              b[1] = nodes[i];
              can_frame frm = bytes_to_can_frame(b, 2, nodes[0]);
              can0.sendMessage(&frm);
            }                
            break;
          }
          case 'e': {
            //Get average energy consumption at desk <i> since the last system restart.
            break;
          }
          case 'v': {
            //Get average visibility error at desk <i> since last system restart.
            break;
          }
          case 'f': {
            //Get the average flicker error on desk <i> since the last system restart.
            break;
          }
          default:
            break;
        }
        break;
      }
      case 'd' : {
        //Set directly the duty cycle of the LED at luminaire i
        uint8_t i = cmd.pop() - '0';
        int val = 0;
        do{
            char c = cmd.pop();
            val = val*10 + (c - '0');
        }
        while(!cmd.is_empty());
        //Do something with the val
        break;
      }
      case 'r' : {
        //Set the illuminance reference at luminaire i
        can_frame frm;

        uint8_t i = cmd.pop() - '0';
        float val = 0.0;
        bool got_decimals = false;
        do{
            char c = cmd.pop();
            if( c == '.' ){
              got_decimals = true;
              break;
            }
            val = val*10 + (c - '0');
        }
        while(!cmd.is_empty());
        if(got_decimals){
          float decimal = 1.0;
          do{
              decimal /= 10.0;
              char c = cmd.pop();
              val += decimal*(c - '0');
          }
          while(!cmd.is_empty());
        }

        if(!i){
          control.lux_ref = val;
          Serial.print(val);
        }
        // serial.add_print("messages sent\n");

        uint8_t b[7];
        b[0] = CAN_SET_ILLUMINANCE_REF;
        b[1] = nodes[1];
        b[2]=nodes[i];
        memcpy(&b[3], &val, 4);
        frm = bytes_to_can_frame(b, 7, nodes[0]);
        can0.sendMessage(&frm);        
        delay(1);
        b[0] = CAN_SET_ILLUMINANCE_REF;
        b[1] = nodes[2];
        b[2]=nodes[i];
        memcpy(&b[3], &val, 4);
        frm = bytes_to_can_frame(b, 7, nodes[0]);
        can0.sendMessage(&frm);
        node_state=NODE_DISCUSSING;
        // Serial.print("reference set, messages sent\n");
        
        break;
      }
      case 'o' : {
        //Set current occupancy state at desk <i>
        uint8_t i = cmd.pop() - '0';
        bool val = cmd.pop() - '0';
        break;
      }
      case 'a' : {
        //Set anti-windup state at desk <i>
        uint8_t i = cmd.pop() - '0';
        bool val = cmd.pop() - '0';
        break;
      }
      case 's' : {
        //Start stream of real-time variable <x> of desk <i>. <x> can be “l” or “d”.
        uint8_t i = cmd.pop() - '0';
        char x = cmd.pop();
        if(!i){
          if(x == 'l') stream_l_local = true;
          else if(x == 'd') stream_d_local = true;
        }
        else{
          if(x == 'l'){
            uint8_t b[2];
            b[0] = CAN_START_STREAM_L;
            b[1] = nodes[i];
            can_frame frm = bytes_to_can_frame(b, 2, nodes[0]);
            can0.sendMessage(&frm);
          }
          else if(x == 'd'){
            uint8_t b[2];
            b[0] = CAN_START_STREAM_D;
            b[1] = nodes[i];
            can_frame frm = bytes_to_can_frame(b, 2, nodes[0]);
            can0.sendMessage(&frm);
          }
        }
        break;
      }
      case 'S' : {
        //Stop stream of real-time variable <x> of desk <i>. <x> can be “l” or “d”
        uint8_t i = cmd.pop() - '0';
        char x = cmd.pop();
        if(!i){
          if(x == 'l') stream_l_local = false;
          else if(x == 'd') stream_d_local = false;
        }
        else{
          if(x == 'l'){
            uint8_t b[2];
            b[0] = CAN_STOP_STREAM_L;
            b[1] = nodes[i];
            can_frame frm = bytes_to_can_frame(b, 2, nodes[0]);
            can0.sendMessage(&frm);
          }
          else if(x == 'd'){
            uint8_t b[2];
            b[0] = CAN_STOP_STREAM_D;
            b[1] = nodes[i];
            can_frame frm = bytes_to_can_frame(b, 2, nodes[0]);
            can0.sendMessage(&frm);
          }
        }
        break;
      }           
      default:
        break;     
    }
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
