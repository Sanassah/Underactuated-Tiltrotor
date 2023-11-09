#include <iq_module_communication.hpp> // if error go to libraries manager and download IQ Module Communication

IqSerial ser(Serial1); // Create an IqSerial object named 'ser' for communication with the IQ Module via Serial1 (Pin 1 of the Microcontroller)
BrushlessDriveClient mot(0); // Create an object for controlling the motor 
CoilTemperatureEstimatorClient tmp(0); // Create an object to read motor temperature
VoltageSuperPositionClient voltageSuperposition(0); // Create an object to control the pulsing of the motor

#define CH3 10 // Throttle
#define CH5 12 // Arm/Disarm
#define CH2 11 // Pitch
#define CH1 9 // Roll

float throttleValue; 
float rollValue;    
float pitchValue;
float phase;
float amplitude;
float armStatus;

float zeroAngleTest;
// Function to read a value from a FLYSKY channel (from 1000 to 2000) input with specified limits
float readChannel(float channelInput, float minLimit, float maxLimit) {
  float rawInputValue = pulseIn(channelInput,HIGH); // Read the raw input value
  float inputValue = map(rawInputValue, 990, 1995, 0, 10000) / 10000.0; // Map the read value to 0.0 - 1.0 and convert to float
  return (maxLimit - minLimit) * inputValue + minLimit; // Linear interpolation to the specified limits and return it
  
}

// Function to read the right joystick values and output as angle and amplitude of pulsing voltage
void cartesianToPolar(float x, float y, float& amplitude, float& angle) {
    // Calculate the amplitude (magnitude) using the Pythagorean theorem
    amplitude = sqrt(x*x + y*y);
    amplitude = min(amplitude, 1.75);
    // Calculate the angle (in radians) using the arctangent function (atan2)
    angle = -1*atan2(x, y);
}

void setup() { 
   ser.begin(); // Initialize the IqSerial object
   Serial.begin(115000); // Initialize Serial (for displaying information on the terminal)  
   pinMode(CH3, INPUT);
   pinMode(CH5, INPUT);
   pinMode(CH2, INPUT);
   pinMode(CH1, INPUT);
   float zeroAngle = 0;
   uint8_t frequency = 1;
   ser.set(voltageSuperposition.zero_angle_, zeroAngle);
   ser.set(voltageSuperposition.frequency_, frequency);
}


void loop() {

  //START OF ARMING CODE
  armStatus = pulseIn(CH5, HIGH); // Variable to store the value read from CH5 (transmitter input)
  if (armStatus > 1500){
    throttleValue = -1 * readChannel(CH3, 0, 2.75); // Read the value from channel 3 and make it between 0 and 2.75 when armed
    ser.set(mot.drive_spin_volts_, throttleValue); // Send a voltage command to the motor using the 'ser.set' function
  }
  else
    ser.set(mot.drive_spin_volts_, 0.0f); // Send 0 voltage to the motor using the 'ser.set' function
//END OF ARMING CODE
  
  


  rollValue = readChannel(CH1, -1.75, 1.75);
  pitchValue = readChannel(CH2, -1.75, 1.75);
  //Serial.println("roll");
  //Serial.println(rollValue);
  //Serial.println("pitch");
  //Serial.println(pitchValue);
  cartesianToPolar(rollValue, pitchValue, amplitude, phase);
  ser.set(voltageSuperposition.phase_, phase); 
  ser.set(voltageSuperposition.amplitude_, amplitude); 

  Serial.println("Phase");
  Serial.println(phase);


  Serial.println("throttle"); 
  Serial.println(throttleValue); 
  float temperature = 0.0f; //Print Temperature of the motor
  ser.get(tmp.t_coil_, temperature);
  Serial.println("Temperature:");
  Serial.println(temperature); 
}
