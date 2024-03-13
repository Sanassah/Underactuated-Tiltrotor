#include <iq_module_communication.hpp>
//Define VERTIQ objects
IqSerial serL(Serial1);  // Create an IqSerial object named 'ser' for communication with the IQ Module via Serial1 and Serial2 (RX1/TX1 pins and RX2/TX2 pins)
IqSerial serR(Serial2);
SerialInterfaceClient sicL(0);
SerialInterfaceClient sicR(0);
BrushlessDriveClient motL(0);  // Create an object for controlling the motor left
BrushlessDriveClient motR(0);  // Create an object for controlling the motor right
VoltageSuperPositionClient voltageSuperpositionL(0);
VoltageSuperPositionClient voltageSuperpositionR(0);
MultiTurnAngleControlClient multL(0);
MultiTurnAngleControlClient multR(0);
BuzzerControlClient buzL(0);
BuzzerControlClient buzR(0);
 
#include <Wire.h>      //I2c communication
#include <SPI.h>       //SPI communication
#include <PWMServo.h>  //Commanding any extra actuators, installed with teensyduino installer
#include "src\MPU6050\MPU6050.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
 
 
 
//========================================================================================================================//
 
 
 
#define USE_PWM_RX //receiver type
#define USE_MPU6050_I2C //Default IMU
#define GYRO_250DPS //Default full scale gyro range (deg/sec
#define ACCEL_2G //Default full scale accelerometer range (G's)
//Setup gyro and accel full scale value selection and scale factor
#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
 
Adafruit_BNO055 bnoR = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bnoL = Adafruit_BNO055(55, 0x28, &Wire2);
MPU6050 mpu6050_C(0x68, &Wire1);
 
sensors_event_t orientationDataL, angVelocityDataL;
sensors_event_t orientationDataR, angVelocityDataR;
 
//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//
 

float phaseR = -1.15;  // Phase of the pulsing angle for arm control pitch -0.75
float phaseL = -1.15;  // Phase of the pulsing angle for arm control pitch -1.45
bool armedFly = false;
// Define PWM pulse width ranges for each flight mode
const int verticalMin = 900;
const int verticalMax = 1100;
const int transitionMin = 1400;
const int transitionMax = 1600;
const int horizontalMin = 1900;
const int horizontalMax = 2100;
// Define flight modes
enum FlightMode { HORIZONTAL, TRANSITION, VERTICAL };
 
float pitch_PID_R, pitch_PID_L = 0;
 
//Radio communication:
long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
 
//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1500;  //roll
unsigned long channel_2_fs = 1500;  //pitch
unsigned long channel_3_fs = 1000;  //thro
unsigned long channel_4_fs = 1500;  //yaw rate
unsigned long channel_5_fs = 1000;  //arm/disarm
unsigned long channel_6_fs = 2000;  //flight mode
 
//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter 0.04
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
 
// Error values for the Center IMU - Initialize to 0, update with actual values
float AccErrorX_C = -0.0379, AccErrorY_C = -0.0307, AccErrorZ_C = 0.6691;
float GyroErrorX_C = 0.5973, GyroErrorY_C = 2.1158, GyroErrorZ_C = 0.7452;
 
//Motors angle Controller
float i_limit = 20;                 //Integrator saturation level, mostly for safety Without guard: 0.8 With Guard:
float arm_angle_kp = 0.8;           //Pitch P-gain - angle mode Without guard: 0.75  With guard: 1 0.8
float arm_angle_Ki = 0.25;          //Pitch I-gain - angle mode Without guard: 0.02   With guard:
float arm_angle_kd = -2.7;         //Pitch D-gain - angle mode Without guard: -0.05   With guard: -3
 
 
//Controller parameters (take note of defaults before modifying!):
float Kp_roll_angle = 1.5;    //Roll P-gain - angle mode 2
float Ki_roll_angle = 0;      //Roll I-gain - angle mode 0.3
float Kd_roll_angle = 0.3;    //Roll D-gain - angle mode (has no effect on controlANGLE2)) 0.5
 
// Yaw PID values
float Kp_yaw = 20;      //Yaw P-gain 3
float Ki_yaw = 0;     //Yaw I-gain 0.05
float Kd_yaw = 0.00;  //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!) 0.0015
 
 
//========================================================================================================================//
//                                                     DECLARE PINS                                                       //
//========================================================================================================================//
 
 
 
#define ch1Pin 10   // Roll Command
#define ch2Pin 11   // Pitch Command
#define ch3Pin 4   // Throttle Command
#define ch4Pin 5   // Yaw Rate Command
#define ch5Pin 12   // Arm/Disarm Command
#define ch6Pin 9   // Vertical/Horizontal Command
 
//PWM servo:  //CHANGE THE PINS NUMBER
const int servoLPin = 23;  //left aileron
const int servoRPin = 22;  //right aileron
const int servoEPin = 21;  //elevator
PWMServo servoL;           //Create servo objects to control a servo or ESC with PWM
PWMServo servoR;
PWMServo servoE;
 
 
 
//========================================================================================================================//
 
 
 
//DECLARE GLOBAL VARIABLES
 
//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;
 
 
//IMU:
float AccX, AccY, AccZ; // For calibration
float GyroX, GyroY, GyroZ; // For calibration
float AccX_L, AccY_L, AccZ_L, AccX_R, AccY_R, AccZ_R,AccX_C, AccY_C, AccZ_C;
float AccX_prev_L, AccY_prev_L, AccZ_prev_L, AccX_prev_R, AccY_prev_R, AccZ_prev_R, AccX_prev_C, AccY_prev_C, AccZ_prev_C;
float GyroX_L, GyroY_L, GyroZ_L, GyroX_R, GyroY_R, GyroZ_R, GyroX_C, GyroY_C, GyroZ_C;
float GyroX_prev_L, GyroY_prev_L, GyroZ_prev_L, GyroX_prev_R, GyroY_prev_R, GyroZ_prev_R, GyroX_prev_C, GyroY_prev_C, GyroZ_prev_C;
 
// Euler angles for each IMU
float pitch_IMU_L_prev;
float roll_IMU_L, pitch_IMU_L, yaw_IMU_L;
float prev_pitch_IMU_L = 0.0;
float roll_IMU_R, pitch_IMU_R, yaw_IMU_R;
float prev_pitch_IMU_R = 0.0;
float roll_IMU_C, pitch_IMU_C, yaw_IMU_C;
// Quaternion components for each IMU
float q0_L = 1.0f, q1_L = 0.0f, q2_L = 0.0f, q3_L = 0.0f;
float q0_R = 1.0f, q1_R = 0.0f, q2_R = 0.0f, q3_R = 0.0f;
float q0_C = 1.0f, q1_C = 0.0f, q2_C = 0.0f, q3_C = 0.0f;
 
//Normalized desired state:
float arm_angle_des_L = 0;  // Angle desired of motor
float arm_angle_des_R = 0;
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;
 
//Max and min to have a good plot:
float maxPlot = -100;
float minPlot = 0;
 
//Controller:
float arm_angle_error, arm_angle_error_integral_L, arm_angle_error_integral_R, arm_angle_error_integral_prev_L, arm_angle_error_integral_prev_R, error_derivative_L,error_derivative_R, pulseAmplitude_scaled_L, pulseAmplitude_scaled_R = 0;
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
float arm_angle_error_L, arm_angle_error_R;
//Mixer
float mL_command_scaled, mR_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float sL_command_scaled, sR_command_scaled, sE_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int sL_command_PWM, sR_command_PWM, sE_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;
 
 
 
//========================================================================================================================//
//                                                      VOID SETUP                                                        //
//========================================================================================================================//
 
 
 
void setup() {
  serL.begin(921600);    // Initialize the IqSerial object
  serR.begin(921600);    // Initialize the IqSerial object
  Serial.begin(921600);  // Initialize Serial (for displaying information on the terminal)
 

  //Initialize all pins
  pinMode(13, OUTPUT);                  //Pin 13 LED blinker on board, do not modify
  servoL.attach(servoLPin, 900, 2100);  //Pin, min PWM value, max PWM value
  servoR.attach(servoRPin, 900, 2100);
  servoE.attach(servoEPin, 900, 2100);
 
  //Initialize radio communication
  radioSetup();
 
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;
 
  //Initialize IMU communication
  IMUinit();
 



  //Get Center IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  // calculate_IMU_error(mpu6050_C); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
 
  //Arm servo channels
  servoL.write(-90);  //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servoR.write(90);  //Set these to 90 for servos if you do not want them to briefly max out on startup
  servoE.write(135);  //Keep these at 0 if you are using servo outputs for motors
 
  //Uncomment to setup the angle of the motors by mechanically putting the motor screw in direction of nose (variable to change is phaseL or phaseR in this function below)
  //setupZeroAngle();
}
 
 
void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;
  loopBlink();  //Indicate we are in main loop with short blink every 1.5 seconds
 
  //Print data at 100hz
  printPlotterData();      //Prints Data with the structure for serial plotter
  // printDataStreamer();      //Prints Data with the structure for Excel

 
  // Get arming status
  armedStatus();  //Check if the throttle cut is off and throttle is low.
 
  // Get flight mode
  flightMode(); //Check if the drone is set to vertical or horizontal mode.
 
 
  //Get vehicle state
  getIMUdata();  //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
 
  //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  Madgwick6_R();
  Madgwick6_L();  
  Madgwick6_C(GyroX_C, -GyroY_C, -GyroZ_C, -AccX_C, AccY_C, AccZ_C, dt);
 
  getDesState();  //Convert raw commands to normalized values based on saturated control limits
 
  //PID Controller
  controlANGLE();     //Stabilize on angle setpoint
 
  //Actuator mixing and scaling to PWM values
  controlMixer();   //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  // scaleCommands();  //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
 
  //Throttle cut check
  throttleCut();  //Directly sets motor commands to low based on state of ch5
  servoL.write(-90);  //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servoR.write(90);  //Set these to 90 for servos if you do not want them to briefly max out on startup
  servoE.write(90);
  //Command actuators
  commandMotors();

  // servoL.write(sL_command_PWM);  //Writes PWM value to servo object
  // servoR.write(sR_command_PWM);
  // servoE.write(sE_command_PWM);
 
  getCommands();
  failSafe();  //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
  //getCalibrationData(); //for left and right sensors

  //Regulate loop rate
  loopRate(2000);  //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}
 
void IMUinit() {  //DESCRIPTION: Initialize IMU
    Wire1.begin();
    Wire1.setClock(1000000);
    mpu6050_C.initialize();
    if (mpu6050_C.testConnection() == false) {
    Serial.println("Middle MPU6050 initialization unsuccessful");
    Serial.println("Check middle MPU6050 wiring or try cycling power");
    while (1) {}
  }
 
  mpu6050_C.setFullScaleGyroRange(GYRO_SCALE);
  mpu6050_C.setFullScaleAccelRange(ACCEL_SCALE);
 
    /* Initialise the sensor */
    if(!bnoR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 Right detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   if(!bnoL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 Left detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
 
  bnoL.setExtCrystalUse(true);
  bnoR.setExtCrystalUse(true);


  

}
 
void calculate_IMU_error(MPU6050 &mpu) {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. The vehicle should be powered up on a flat surface.
  int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
  float AccErrorX = 0.0, AccErrorY = 0.0, AccErrorZ = 0.0;
  float GyroErrorX = 0.0, GyroErrorY = 0.0, GyroErrorZ = 0.0;
 
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
 
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
 
 
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;

  }
  Serial.println();
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;
 
  //Print out the calculated errors for manual recording
  Serial.println("Calculated IMU Errors:");
  Serial.print("AccErrorX = ");
  Serial.println(AccErrorX, 4);
  Serial.print("AccErrorY = ");
  Serial.println(AccErrorY, 4);
  Serial.print("AccErrorZ = ");
  Serial.println(AccErrorZ, 4);
  Serial.print("GyroErrorX = ");
  Serial.println(GyroErrorX, 4);
  Serial.print("GyroErrorY = ");
  Serial.println(GyroErrorY, 4);
  Serial.print("GyroErrorZ = ");
  Serial.println(GyroErrorZ, 4);
 
  // Reminder message
  Serial.println("Record these values and use them to correct sensor readings in getIMUdata().");
}
 
 
void setupZeroAngle() {  //Setup the right angle for the 0 angle of the motors to be in front of pitch
 
  //serL.set(voltageSuperpositionL.sample_mechanical_zero_);
  //serR.set(voltageSuperpositionR.sample_mechanical_zero_);

  //serL.get(voltageSuperpositionL.zero_angle_, zeroAngleL);
  //serR.get(voltageSuperpositionR.zero_angle_, zeroAngleR);
  // Serial.print("Zero Angle Left: ");
  // Serial.print(zeroAngleL);
  // Serial.print(" Zero Angle Right: ");
  // Serial.println(zeroAngleR);
  // Serial.println();
}
 
 
void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate);  //Pin 13 is built in LED
 
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}
 
void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_5_pwm < 1500) && (channel_3_pwm < 1050)) {
    armedFly = true;
  }
}
 
void flightMode() {
  //DESCRIPTION: Check if the tiltrotor is set to vertical or horizontal flight mode.
  FlightMode mode;
  if (channel_6_pwm >= horizontalMin && channel_6_pwm <= horizontalMax) {
    mode = HORIZONTAL;
  } else if (channel_6_pwm >= transitionMin && channel_6_pwm <= transitionMax) {
    mode = TRANSITION;
  } else if (channel_6_pwm >= verticalMin && channel_6_pwm <= verticalMax) {
    mode = VERTICAL;
  } else {
    // Unknown mode or no signal
    mode = VERTICAL; // Default to vertical mode
  }
  // Print flight mode
  // switch (mode) {
  //   case HORIZONTAL:
  //     Serial.println("Flight mode: Horizontal");
  //     break;
  //   case TRANSITION:
  //     Serial.println("Flight mode: Transition");
  //     break;
  //   case VERTICAL:
  //     Serial.println("Flight mode: Vertical");
  //     break;
  // }
  // delay(100); // Adjust delay as needed
}

 
void getIMUdata() {
 
  bnoL.getEvent(&orientationDataL, Adafruit_BNO055::VECTOR_EULER);
  bnoL.getEvent(&angVelocityDataL, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bnoR.getEvent(&orientationDataR, Adafruit_BNO055::VECTOR_EULER);
  bnoR.getEvent(&angVelocityDataR, Adafruit_BNO055::VECTOR_GYROSCOPE);
 
 
  int16_t AcX_C, AcY_C, AcZ_C, GyX_C, GyY_C, GyZ_C;
  // Process data from the Center IMU
  mpu6050_C.getMotion6(&AcX_C, &AcY_C, &AcZ_C, &GyX_C, &GyY_C, &GyZ_C);
  AccX_C = (AcX_C / ACCEL_SCALE_FACTOR) - AccErrorX_C;
  AccY_C = (AcY_C / ACCEL_SCALE_FACTOR) - AccErrorY_C;
  AccZ_C = (AcZ_C / ACCEL_SCALE_FACTOR) - AccErrorZ_C;
  GyroX_C = (GyX_C / GYRO_SCALE_FACTOR) - GyroErrorX_C;
  GyroY_C = (GyY_C / GYRO_SCALE_FACTOR) - GyroErrorY_C;
  GyroZ_C = (GyZ_C / GYRO_SCALE_FACTOR) - GyroErrorZ_C;
  // Apply low-pass filter for Center IMU
  AccX_C = (1.0 - B_accel) * AccX_prev_C + B_accel * AccX_C;
  AccY_C = (1.0 - B_accel) * AccY_prev_C + B_accel * AccY_C;
  AccZ_C = (1.0 - B_accel) * AccZ_prev_C + B_accel * AccZ_C;
  GyroX_C = (1.0 - B_gyro) * GyroX_prev_C + B_gyro * GyroX_C;
  GyroY_C = (1.0 - B_gyro) * GyroY_prev_C + B_gyro * GyroY_C;
  GyroZ_C = (1.0 - B_gyro) * GyroZ_prev_C + B_gyro * GyroZ_C;
  // Update previous values for Center IMU
  AccX_prev_C = AccX_C;
  AccY_prev_C = AccY_C;
  AccZ_prev_C = AccZ_C;
  GyroX_prev_C = GyroX_C;
  GyroY_prev_C = GyroY_C;
  GyroZ_prev_C = GyroZ_C;
}
 
void Madgwick6_L() {
    GyroX_L = angVelocityDataL.gyro.x;
    GyroY_L = angVelocityDataL.gyro.y;
    GyroZ_L = angVelocityDataL.gyro.z;
    roll_IMU_L = orientationDataL.orientation.x; //degrees
    pitch_IMU_L = orientationDataL.orientation.y; //degrees
    yaw_IMU_L = orientationDataL.orientation.z; //degrees
    if (prev_pitch_IMU_L == 0) {       
      prev_pitch_IMU_L = pitch_IMU_L;     } 
    else {    
      // Check for sudden change in pitch
      if (abs(pitch_IMU_L - prev_pitch_IMU_L) > 5) {
        // If the change is sudden, ignore the new value and use the previous one
        pitch_IMU_L = prev_pitch_IMU_L;
      }
    }
    // Update previous pitch value for next iteration
    prev_pitch_IMU_L = pitch_IMU_L;
 }
 
void Madgwick6_R() {
    GyroX_R = angVelocityDataR.gyro.x;
    GyroY_R = angVelocityDataR.gyro.y; //*-1
    GyroZ_R = angVelocityDataR.gyro.z;
    roll_IMU_R = orientationDataR.orientation.x; //degrees
    pitch_IMU_R = orientationDataR.orientation.y; //degrees  *-1
    yaw_IMU_R = orientationDataR.orientation.z; //degrees
    if (prev_pitch_IMU_R == 0) {       
      prev_pitch_IMU_R = pitch_IMU_R;     } 
    else {    
      // Check for sudden change in pitch
      if (abs(pitch_IMU_R - prev_pitch_IMU_R) > 5) {
        // If the change is sudden, ignore the new value and use the previous one
        pitch_IMU_R = prev_pitch_IMU_R;
      }
    }

    // Update previous pitch value for next iteration
    prev_pitch_IMU_R = pitch_IMU_R;
  }
 
void Madgwick6_C(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
 
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
 
  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;
 
  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1_C * gx - q2_C * gy - q3_C * gz);
  qDot2 = 0.5f * (q0_C * gx + q2_C * gz - q3_C * gy);
  qDot3 = 0.5f * (q0_C * gy - q1_C * gz + q3_C * gx);
  qDot4 = 0.5f * (q0_C * gz + q1_C * gy - q2_C * gx);
 
  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
 
    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0_C;
    _2q1 = 2.0f * q1_C;
    _2q2 = 2.0f * q2_C;
    _2q3 = 2.0f * q3_C;
    _4q0 = 4.0f * q0_C;
    _4q1 = 4.0f * q1_C;
    _4q2 = 4.0f * q2_C;
    _8q1 = 8.0f * q1_C;
    _8q2 = 8.0f * q2_C;
    q0q0 = q0_C * q0_C;
    q1q1 = q1_C * q1_C;
    q2q2 = q2_C * q2_C;
    q3q3 = q3_C * q3_C;
 
    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_C - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2_C + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3_C - _2q1 * ax + 4.0f * q2q2 * q3_R - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
 
    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }
 
  //Integrate rate of change of quaternion to yield quaternion
  q0_C += qDot1 * invSampleFreq;
  q1_C += qDot2 * invSampleFreq;
  q2_C += qDot3 * invSampleFreq;
  q3_C += qDot4 * invSampleFreq;
 
  //Normalise quaternion
  recipNorm = invSqrt(q0_C * q0_C + q1_C * q1_C + q2_C * q2_C + q3_C * q3_C);
  q0_C *= recipNorm;
  q1_C *= recipNorm;
  q2_C *= recipNorm;
  q3_C *= recipNorm;
 
  //Compute angles
  roll_IMU_C = atan2(q0_C * q1_C + q2_C * q3_C, 0.5f - q1_C * q1_C - q2_C * q2_C) * 57.29577951;
  pitch_IMU_C = -asin(constrain(-2.0f * (q1_C*q3_C - q0_C*q2_C),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU_C = -atan2(q1_C * q2_C + q0_C * q3_C, 0.5f - q2_C * q2_C - q3_C * q3_C) * 57.29577951;
}
 
void getDesState() {
 
  thro_des = constrain((channel_3_pwm - 1000.0) / 143, 0, 7);          //Between 0 and 7V for 2314 VERTIQ motor
  yaw_des = constrain((channel_4_pwm - 1500.0) / 17, -30, 30);     //Between -30 degs and 30 deg/s but damped
  roll_des = constrain((channel_1_pwm - 1500.0) / 50, -10, 10);        //Between -10 and 10
  //arm_angle_des = constrain((channel_2_pwm - 1500) * 0.18 - 45, -47, 47); //Between -47 is vertical and 43 is horizontal
  arm_angle_des_L = -1*constrain((channel_2_pwm - 1500) * 0.02 - 44.5, -57, -37); //Between -47 is vertical and 43 is horizontal
  arm_angle_des_R = -1*constrain((channel_2_pwm - 1500) * 0.02 - 44.5, -57, -37); //Between -47 is vertical and 43 is horizontal
}
 
 
void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
 
  //Roll
  error_roll = roll_des - roll_IMU_C;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_3_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX_C;
  roll_PID = 0.05 * (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll - Kd_roll_angle * derivative_roll);  //Scaled by .01 to bring within -1 to 1 range 0.05
 
 
 
  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - 10*GyroZ_L -3; //GyroZ_C
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_3_pwm < 1060) {  //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit);  //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;

  yaw_PID = .01 * (Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw);  //Scaled by .01 to bring within -1 to 1 range
  yaw_PID = constrain(yaw_PID, -5, 5);
 
  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
 
  controlArmAngle();
}

void getCalibrationData(){
      if (current_time - print_counter > 10000) {
    print_counter = micros();
  uint8_t sysL, gyroL, accelL, magL = 0;
  uint8_t sysR, gyroR, accelR, magR = 0;
  bnoL.getCalibration(&sysL, &gyroL, &accelL, &magL);
  Serial.print(F("CalibrationL: "));
  Serial.print(sysL, 4);
  Serial.print(F(" "));
  Serial.print(gyroL, 4);
  Serial.print(F(" "));
  Serial.print(accelL, 4);
  Serial.print(F(" "));
  Serial.println(magL, 4);   

  bnoR.getCalibration(&sysR, &gyroR, &accelR, &magR);
  Serial.print(F("CalibrationR: "));
  Serial.print(sysR, 4);
  Serial.print(F(" "));
  Serial.print(gyroR, 4);
  Serial.print(F(" "));
  Serial.print(accelR, 4);
  Serial.print(F(" "));
  Serial.println(magR, 4);
      }
}
 
void controlArmAngle() {
  // DESCRIPTION: Computes control commands based on state error (angle) for both left and right motors.
  // Initialize control variables for left and right
 
  arm_angle_error_L = arm_angle_des_L - pitch_IMU_L + yaw_PID; // Left arm angle error entre 3.65 et 3.7 offset -3.65
  arm_angle_error_R = arm_angle_des_R - pitch_IMU_R - yaw_PID; // Right arm angle error 
 
    if (fabs(arm_angle_error_L) <= 0) { // Left arm control 0.2
    pulseAmplitude_scaled_L = 0.0;
  } else {
    arm_angle_error_integral_L = arm_angle_error_integral_prev_L + arm_angle_error_L * dt;
    arm_angle_error_integral_L = constrain(arm_angle_error_integral_L, -i_limit, i_limit);  // Saturate integrator
 
    error_derivative_L = GyroY_L;
    pitch_PID_L = -0.09 * (arm_angle_kp * arm_angle_error_L + arm_angle_Ki * arm_angle_error_integral_L - arm_angle_kd * error_derivative_L);  // Scaled by 0.09
  }
 
  // Update variables for next iteration
  arm_angle_error_integral_prev_L = arm_angle_error_integral_L;
 
  if (fabs(arm_angle_error_R) <= 0) { // Right arm control
      pulseAmplitude_scaled_R = 0.0;
  } else {
      arm_angle_error_integral_R = arm_angle_error_integral_prev_R + arm_angle_error_R * dt;
      arm_angle_error_integral_R = constrain(arm_angle_error_integral_R, -i_limit, i_limit);  // Saturate integrator
 
      error_derivative_R = GyroY_R;
      pitch_PID_R = -0.09 * (arm_angle_kp * arm_angle_error_R + arm_angle_Ki * arm_angle_error_integral_R - arm_angle_kd * error_derivative_R);  // Scaled by 0.09
    }
  arm_angle_error_integral_prev_R = arm_angle_error_integral_R;
 
}
 
void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pulseAmplitude_scaled, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pulseAmplitude_scaled and the back two should have +pulseAmplitude_scaled etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands()
   * in preparation to be sent to the motor ESCs and servos.
   *
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pulseAmplitude_scaled, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */
 
  mL_command_scaled = constrain(thro_des + roll_PID, 0, 7);  //Left motor 
  mR_command_scaled = -1*constrain(thro_des - roll_PID, 0, 7);  //Right motor
  pulseAmplitude_scaled_L = constrain(pitch_PID_L , -thro_des, thro_des); 
  pulseAmplitude_scaled_R = constrain(pitch_PID_R , -thro_des, thro_des);
 
 
}
 
void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated
   * which are used to command the servos.
   */
 
  //Scaled to 0-180 for servo library
  sL_command_PWM = sL_command_scaled * 180;
  sR_command_PWM = sR_command_scaled * 180;
  sE_command_PWM = sE_command_scaled * 180;
 
  //Constrain commands to servos within servo library bounds
  sL_command_PWM = constrain(sL_command_PWM, 0, 180);
  sR_command_PWM = constrain(sR_command_PWM, 0, 180);
  sE_command_PWM = constrain(sE_command_PWM, 0, 180);
}
 
void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
      minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.
 
      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if ((channel_5_pwm > 1500) || (armedFly == false)) {
    armedFly = false;
    mL_command_scaled = 0;
    mR_command_scaled = 0;
    sL_command_PWM = 0;
    sR_command_PWM = 0;
    sE_command_PWM = 0;
  }
}
 
void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();
 
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}
 
void commandMotors() {
  //DESCRIPTION: Send command to motor via IQ library
  serL.set(motL.drive_spin_volts_, mL_command_scaled);  // Send a voltage command to the motor using the 'ser.set' function
  serL.set(voltageSuperpositionL.phase_, phaseL);
  serL.set(voltageSuperpositionL.amplitude_, -1*pulseAmplitude_scaled_L);
 
  serR.set(motR.drive_spin_volts_, mR_command_scaled);
  serR.set(voltageSuperpositionR.phase_, phaseR);
  serR.set(voltageSuperpositionR.amplitude_, -1*pulseAmplitude_scaled_R);
}
 
void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise.
   */
 
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);
 
  //Low-pass the critical commands and update previous values
  float b = 0.7;  //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
 
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
 
}
 
void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;
 
  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;
 
  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}
 
 
//=========================================================================================//
 
//HELPER FUNCTIONS
 
float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  return 1.0 / sqrtf(x);  //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}
 
int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
 
void printPlotterData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
 
       Serial.print("-X"); Serial.print(":"); Serial.println("10");
       Serial.print("X"); Serial.print(":"); Serial.println("-10");
 
      //  Serial.print("Channel1"); Serial.print(":"); Serial.println(channel_1_pwm);
      //  Serial.print("Channel2"); Serial.print(":"); Serial.println(channel_2_pwm);
      //  Serial.print("Channel3"); Serial.print(":"); Serial.println(channel_3_pwm);
      //  Serial.print("Channel4"); Serial.print(":"); Serial.println(channel_4_pwm);
      //  Serial.print("Channel5"); Serial.print(":"); Serial.println(channel_5_pwm);
      //  Serial.print("Channel6"); Serial.print(":"); Serial.println(channel_6_pwm);
       
       
        // Serial.print("RollAngle"); Serial.print(":"); Serial.println(roll_IMU_C);
        // Serial.print("RollPIDCorrection"); Serial.print(":"); Serial.println(-roll_PID*100);
        // Serial.print("PitchError"); Serial.print(":"); Serial.println(pitch_IMU_R-pitch_IMU_L);
        // Serial.print("PIDYaw"); Serial.print(":"); Serial.println(yaw_PID);
        // Serial.print("PitchC"); Serial.print(":"); Serial.println(pitch_IMU_C);

      // Serial.print("PitchRateL"); Serial.print(":"); Serial.println(GyroY_L);
      // Serial.print("PitchRateR"); Serial.print(":"); Serial.println(GyroY_R);
      Serial.print("GyroZ_C"); Serial.print(":"); Serial.println(GyroZ_C);
      Serial.print("froombno055"); Serial.print(":"); Serial.println(GyroZ_L);
      Serial.print("yaw_PID"); Serial.print(":"); Serial.println(yaw_PID);
      Serial.print("yawDesired"); Serial.print(":"); Serial.println(yaw_des);
 
       //Serial.print("Integral error"); Serial.print(":"); Serial.println(arm_angle_error_integral_R);
     
      //  Serial.print("YawRate"); Serial.print(":"); Serial.println(GyroZ_C);
      //  Serial.print("Roll Center"); Serial.print(":"); Serial.println(roll_IMU_C);
       //Serial.print("Roll"); Serial.print(":"); Serial.println(roll_IMU_C);
       //Serial.print("PulseL"); Serial.print(":"); Serial.println(pulseAmplitude_scaled_L,4);
       //Serial.print("PulseR"); Serial.print(":"); Serial.println(pulseAmplitude_scaled_R,4);
     
  }
}

void  printDataStreamer() { // Excel Data Streamer
     if (current_time - print_counter > 10000) {
    print_counter = micros();

      // Serial.print(GyroX_L); Serial.print(",");
      // Serial.print(GyroY_L); Serial.print(",");
      // Serial.print(GyroZ_L); Serial.print(",");
      // Serial.print(GyroX_R); Serial.print(",");
      // Serial.print(GyroY_R); Serial.print(",");
      // Serial.print(GyroZ_R); Serial.print(",");
      // Serial.print(roll_IMU_C); Serial.print(",");
      // Serial.print(-roll_PID*100); Serial.print(",");
      // Serial.print(roll_des); Serial.print(",");
      // Serial.print(arm_angle_des_L); Serial.print(",");
      // Serial.print(pitch_IMU_R); Serial.print(",");
      // Serial.print(pitch_IMU_L); Serial.print(",");
      // Serial.println();
     }
}

