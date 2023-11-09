//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

//========================================================================================================================//

//This file contains all necessary functions and code used for radio communication to avoid cluttering the main code

unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;
int ppm_counter = 0;
unsigned long time_ms = 0;

const unsigned long MIN_PULSE_WIDTH = 1000;
const unsigned long MAX_PULSE_WIDTH = 2000;

void radioSetup() {

  //PWM Receiver
  #if defined USE_PWM_RX
    //Declare interrupt pins 
    pinMode(ch3Pin, INPUT);
    pinMode(ch5Pin, INPUT);
    pinMode(ch2Pin, INPUT);
    delay(20);
    //Attach interrupt and point to corresponding ISR functions
    attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch5Pin), getCh5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
    delay(20);

  //SBUS Recevier 
  #elif defined USE_SBUS_RX
    sbus.begin();

  //DSM receiver
  #elif defined USE_DSM_RX
    Serial3.begin(115000);
  #else
    #error No RX type defined...
  #endif
}

unsigned long getRadioPWM(int ch_num) {
  //DESCRIPTION: Get current radio commands from interrupt routines 
  unsigned long returnPWM = 0;
  
  if (ch_num == 1) {
    returnPWM = channel_1_raw;
  }
  else if (ch_num == 2) {
    returnPWM = channel_2_raw;
  }
  else if (ch_num == 3) {
    returnPWM = channel_3_raw;
  }
  else if (ch_num == 4) {
    returnPWM = channel_4_raw;
  }
  else if (ch_num == 5) {
    returnPWM = channel_5_raw;
  }
  else if (ch_num == 6) {
    returnPWM = channel_6_raw;
  }
  
  return returnPWM;
}

//For DSM type receivers
void serialEvent3(void)
{
  #if defined USE_DSM_RX
    while (Serial3.available()) {
        DSM.handleSerialEvent(Serial3.read(), micros());
    }
  #endif
}



//========================================================================================================================//



//INTERRUPT SERVICE ROUTINES (for reading PWM and PPM)

void getCh2() {
  int trigger = digitalRead(ch2Pin);
  if(trigger == 1) {
    rising_edge_start_2 = micros();
  }
  else if(trigger == 0) {
    channel_2_raw = micros() - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(ch3Pin);
  if(trigger == 1) {
    rising_edge_start_3 = micros();
  }
  else if(trigger == 0) {
    channel_3_raw = micros() - rising_edge_start_3;
  }
}

void getCh5() {
  int trigger = digitalRead(ch5Pin);
  if(trigger == 1) {
    rising_edge_start_5 = micros();
  }
  else if(trigger == 0) {
    channel_5_raw = micros() - rising_edge_start_5;
  }
}