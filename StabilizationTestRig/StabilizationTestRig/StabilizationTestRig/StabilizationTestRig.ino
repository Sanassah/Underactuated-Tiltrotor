#include <iq_module_communication.hpp>
IqSerial ser(Serial1); // Create an IqSerial object named 'ser' for communication with the IQ Module via Serial1 (Pin 1 of the Microcontroller)
BrushlessDriveClient mot(0); // Create an object for controlling the motor 
VoltageSuperPositionClient voltageSuperposition(0);
MultiTurnAngleControlClient mult(0);


#define ch3Pin 10 // Throttle Command
#define ch2Pin 11 // Pitch Angle Commannd
#define ch5Pin 25 // Arm/Disarm Command


float phase=0;
bool armedFly = false;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;


//Uncomment only one receiver type
#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

//Uncomment only one IMU
#define USE_MPU6050_I2C //Default
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//



//REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <MPU6050.h>


#if defined USE_MPU6050_I2C
  #include "C:\Users\alger\OneDrive - Concordia University - Canada\Desktop\MECH 490\ArduinoCodes\StabilizationTestRig\StabilizationTestRig\src\MPU6050\MPU6050.h"
  MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include "C:\Users\alger\OneDrive - Concordia University - Canada\Desktop\MECH 490\ArduinoCodes\StabilizationTestRig\StabilizationTestRig\src/MPU9250/MPU9250.h"
  MPU9250 mpu9250(SPI2,36);
#else
  #error No MPU defined... 
#endif



//========================================================================================================================//



//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#endif
  
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//



//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1500; //pitch angle
unsigned long channel_2_fs = 1500; //NOT USED
unsigned long channel_3_fs = 1000; //thro
unsigned long channel_4_fs = 1500; //NOT USED
unsigned long channel_5_fs = 1000; //Arm
unsigned long channel_6_fs = 2000; //NOT USED

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode


float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)


float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)


//========================================================================================================================//



//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;


//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float pitch_des = 0;// Angle desired of motor
float thro_des = 0;
float pitch_passthru;

//Max and min to have a good plot:
float maxPlot = -100;
float minPlot = 0;

//Controller:
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;

//Mixer
float amplitude_scaled;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  ser.begin(115200); // Initialize the IqSerial object
  Serial.begin(115200); // Initialize Serial (for displaying information on the terminal)  

  //Initialize radio communication
  radioSetup();

  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.
  
  //Initialize IMU communication
  IMUinit();

  //Uncomment to setup the angle of the motors, place the metal screw in the direction of the drone and reupload code
  // setupZeroAngle(); 
}


void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands();
  //printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
  
  // Get arming status
  armedStatus(); //Check if the throttle cut is off and throttle is low.

  //Get vehicle state
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  getDesState(); //Convert raw commands to normalized values based on saturated control limits

  //PID Controller - SELECT ONE:
  controlANGLE(); //Stabilize on angle setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here

  //Throttle cut check
  throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors();

  getCommands();

  //Serial.print("Desired Angle");
  Serial.print(-1*pitch_des);
  Serial.print(",");
  //Serial.print("Actual Angle");
  Serial.print(-1*pitch_IMU);
  Serial.print(",");
  Serial.print(thro_des);
  Serial.print(",");
  Serial.print(amplitude_scaled);


  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}


// Function to read the right joystick values and output as angle and amplitude of pulsing voltage
void cartesianToPolar(float x, float y, float& amplitude, float& angle) {
    // Calculate the amplitude (magnitude) using the Pythagorean theorem
    amplitude = sqrt(x*x + y*y);
    amplitude = min(amplitude, 1.75);
    // Calculate the angle (in radians) using the arctangent function (atan2)
    angle = -1*atan2(x, y);
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
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}


void IMUinit() {
  //DESCRIPTION: Initialize IMU
  /*
   * Don't worry about how this works.
   */
  #if defined USE_MPU6050_I2C
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    
    mpu6050.initialize();
    
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    
  #elif defined USE_MPU9250_SPI
    int status = mpu9250.begin();    

    if (status < 0) {
      Serial.println("MPU9250 initialization unsuccessful");
      Serial.println("Check MPU9250 wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu9250.setGyroRange(GYRO_SCALE);
    mpu9250.setAccelRange(ACCEL_SCALE);
    mpu9250.setMagCalX(MagErrorX, MagScaleX);
    mpu9250.setMagCalY(MagErrorY, MagScaleY);
    mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
    mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
  #endif
}


void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  #if defined USE_MPU6050_I2C
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  #endif

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
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
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  //use 6DOF algorithm if MPU6050 is being used
  #if defined USE_MPU6050_I2C 
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  #endif
  
  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
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
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //compute angles - NWU
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees


}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
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
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
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

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  
  integral_pitch = 0; //integral_pitch_prev + error_pitch*dt;
  //integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  
  derivative_pitch = GyroY;
  pitch_PID = -.04*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Update pitch variables
  //integral_pitch_prev = integral_pitch;
}

void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */
  
  // Constrain the pitch control signal (pitch_PID) to be within the range of +/- amplitude
  pitch_PID = constrain(pitch_PID, -thro_des, thro_des);

  //test rig mixing
  amplitude_scaled = pitch_PID; // Only the angle of the propeller changes



}


void commandMotors() {
  //DESCRIPTION: Send command to motor via IQ library

  //START OF ARMING CODE

  thro_des = -1*thro_des;
  ser.set(mot.drive_spin_volts_, thro_des); // Send a voltage command to the motor using the 'ser.set' function


  ser.set(voltageSuperposition.phase_, phase);
  ser.set(voltageSuperposition.amplitude_, amplitude_scaled); 

}

//=========================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F(" AccY:"));
    Serial.print(AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX:"));
    Serial.print(MagX);
    Serial.print(F(" MagY:"));
    Serial.print(MagY);
    Serial.print(F(" MagZ:"));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();

    Serial.println(F(" pitch_PID:"));
    Serial.print(pitch_PID);

  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.println(F("Arm status "));
    Serial.println(channel_5_pwm);
    Serial.println(F("amplitude_scaled:  "));
    Serial.println(amplitude_scaled);
    Serial.println(F("Phase:  "));
    Serial.println(phase);
  }
}


void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  #if defined USE_PPM_RX || defined USE_PWM_RX
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_5_pwm = getRadioPWM(5);
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  pitch_des = -1*constrain((channel_2_pwm - 1495.0)/5.56, 0, 90); //Between 0 and -90 degrees
  thro_des = (channel_3_pwm - 1000.0)/363.0; //Between -1 and 1

}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_5_pwm < 1500) && (channel_3_pwm < 1050)) {
    armedFly = true;
  }
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
    thro_des = 0;

  }
}

void setupZeroAngle() {
  float zeroAngle = 0;
  ser.set(voltageSuperposition.zero_angle_, zeroAngle);
  ser.save(voltageSuperposition.zero_angle_);
}