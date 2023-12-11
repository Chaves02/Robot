// Spider_mini (Top View)
//  -----               -----
// |  L3 |             |  L1 |
// | GP4 |             | GP0 |
//  ----- -----   ----- -----
//       |     | |     |
//       | GP5 | | GP1 |
//        -----   -----
//       |     | |     |
//       | GP6 | | GP2 |
//  ----- -----   ----- -----
// |  L4 |             |  L2 |
// | GP7 |             | GP3 |
//  -----               -----

#include <Arduino.h>
#include <Servo.h>
#include <elapsedMillis.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

#define MAX_DISTANCE 300 // Maximum distance in milimeters

//Create an instance of the VL53L0X library
VL53L0X VL53L0X_sensor;

MPU6050 mpu(0x68); // MPU6050 object

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

elapsedMillis timeElapsed; 
int flag = 0; // Flag for checking if an object is detected
const long interval = 500;  // Interval for checking the sensor (in milliseconds)

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=0.8, Ki=5, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

typedef enum{
  Front,
  Right,
  Left,
  Back
} state;

state currentState = Front;
int side = 0;
int aux = 0;

const int numberOfServos = 8; // Number of servos
const int numberOfACE = 9; // Number of action code elements
int servoCal[] = { -2, -4, 2, -7, -5, 0, 0, 5 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoPrevPrg[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo previous prg
int servoPrgPeriod = 20; // 50 ms
Servo servo[numberOfServos]; // Servo object

// Zero
int ZeroStep = 1;
int Zero [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {  0,  45, 135, 180, 180, 135,  45,  0, 1000  }, // zero position          /////////check///////////
};

// Standby
int servoPrg01step = 2;
int servoPrg01 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  200  }, // prep standby       /////////check///////////
  {  -20,   0,   0,  20,  20,   0,   0, -20,  200  }, // standby
};

// Forward
int ForwardStep = 11;
int Forward [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby               ////////check//////
  {   20,   0,   0,   0,   0,   0, -45,  20,  100  }, // leg1,4 up; leg4 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0, -57,  57,   0,   0,   0,  45,   0,  100  }, // leg1,4 bk; leg2 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,  57,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up; leg1 fw
  {    0,   0, -57,   0,   0,  45,   0,   0,  100  }, // leg2,3 bk
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0,   0, -20,   0,   0,   0,  100  }, // leg3 up
  {    0,   0,   0,   0,  20, -45,   0,   0,  100  }, // leg3 fw dn
};

// Backward
int BackwardStep = 11;
int Backward [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby                ////////check////////
  {   20, -45,   0,   0,   0,   0,   0,  20,  100  }, // leg4,1 up; leg1 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg3,2 up
  {    0,  45,   0,   0,   0,  65, -65,   0,  100  }, // leg4,1 bk; leg3 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {   20,   0,   0,   0,   0,   0,  65,  20,  100  }, // leg4,1 up; leg4 fw
  {    0,   0,  45,   0,   0, -65,   0,   0,  100  }, // leg3,1 bk
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,   0, -20,   0,   0,   0,   0,  100  }, // leg2 up
  {    0,   0, -45,  20,   0,   0,   0,   0,  100  }, // leg2 fw dn
};

// Move Left
int MoveleftStep = 11;
int Moveleft [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby                  ////////check////////
  {    0,   0, -45, -20, -20,   0,   0,   0,  100  }, // leg3,2 up; leg2 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0,  65,  45,   0,   0, -65,   0,   0,  100  }, // leg3,2 bk; leg1 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,  65,   0,   0,  100  }, // leg3,2 up; leg3 fw
  {    0, -65,   0,   0,   0,   0,  45,   0,  100  }, // leg1,4 bk
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {    0,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg4 up
  {    0,   0,   0,   0,   0,   0, -45, -20,  100  }, // leg4 fw dn
};

// Move Right
int MoverightStep = 11;
int Moveright [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby                ////////check////////
  {    0,   0,   0, -20, -20, -45,   0,   0,  100  }, // leg2,3 up; leg3 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg4,1 up
  {    0,   0, -70,   0,   0,  45,  65,   0,  100  }, // leg2,3 bk; leg4 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,  70, -20, -20,   0,   0,   0,  100  }, // leg2,3 up; leg2 fw
  {    0,  45,   0,   0,   0,   0, -65,   0,  100  }, // leg4,1 bk
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,   0,  100  }, // leg1 up
  {  -20, -45,   0,   0,   0,   0,   0,   0,  100  }, // leg1 fw dn
};

// Turn left
int servoPrg06step = 8;
int servoPrg06 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby           ////////check////////
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0,  45,   0,   0,   0,   0,  45,   0,  100  }, // leg1,4 turn
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0,   0,  45,   0,   0,  45,   0,   0,  100  }, // leg2,3 turn
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {    0, -45, -45,   0,   0, -45, -45,   0,  100  }, // leg1,2,3,4 turn
};

// Turn right
int servoPrg07step = 8;
int servoPrg07 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby           ////////check////////
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0,   0, -45,   0,   0, -45,   0,   0,  100  }, // leg2,3 turn
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0, -45,   0,   0,   0,   0, -45,   0,  100  }, // leg1,4 turn
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,  45,  45,   0,   0,  45,  45,   0,  100  }, // leg1,2,3,4 turn
};

// Lie
int servoPrg08step = 6;
int servoPrg08 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  80,  90,  30, 100 }, //standby                   ///////check/////////
  {  150,   0,   0, -20,   0,   0,   0,  20, 200 }, //leg1 maxup and leg2,4 up
  {    0,  45,   0,   0,   0,   0,   0,   0, 350 }, //leg1 fw
  {    0, -45,   0,   0,   0,   0,   0,   0, 350 }, //leg1 bk
  {    0,  45,   0,   0,   0,   0,   0,   0, 350 }, //leg1 fw
  {    0, -45,   0,   0,   0,   0,   0,   0, 350 }  //leg1 bk
};

// Say Hi
int servoPrg09step = 4;
int servoPrg09 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms                             /////////check////////
  {    0,  90,  90, 150, 180,  90,  90,  30,  200  }, // leg1, 3 down
  {   30,   0,   0,   0, -30,   0,   0,   0,  200  }, // standby
  {  -30,   0,   0,   0,  30,   0,   0,   0,  200  }, // leg1, 3 down
  {   30,   0,   0,   0, -30,   0,   0,   0,  200  }, // standby
};

// Fighting
int servoPrg10step = 11;
int servoPrg10 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms                          /////check/////
  {    0,  90,  90, 180, 150,  90,  90,  30,  200  }, // leg1, 2 down
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -40, -40,   0,   0, -40, -40,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {   30, -20, -20, -20,  30, -20, -20, -20,  200  }, // leg1, 2 up ; leg3, 4 down
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -40, -40,   0,   0, -40, -40,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // leg1, 2 up ; leg3, 4 down
};

// Push up
int servoPrg11step = 11;
int servoPrg11 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms                           /////check/////
  {   30,  45,  38, 150, 150, 135, 147,  30,  300  }, // start position           
  {   30,   0,   0, -40, -30,   0,   0,   0,  400  }, // down
  {  -30,   0,   0,  40,  30,   0,   0,   0,  500  }, // up
  {   30,   0,   0,   0, -30,   0,   0,  40,  600  }, // down
  {  -30,   0,   0,   0,  30,   0,   0, -40,  700  }, // up
  {   30,   0,   0, -40, -30,   0,   0,   0,  1300 }, // down
  {  -30,   0,   0,  40,  30,   0,   0,   0,  1800 }, // up
  {   45,   0,   0, -30, -45,   0,   0,  30,  200  }, // fast down
  {  -45,   0,   0,   0,  10,   0,   0,   0,  500  }, // leg1 up
  {    0,   0,   0,   0,  35,   0,   0,   0,  500  }, // leg2 up
  {    0,   0,   0,  30,   0,   0,   0, -30,  500  }, // leg3, leg4 up
};

// Sleep
int servoPrg12step = 2;
int servoPrg12 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms                     ////check////
  {    0,  90,  90, 150, 150,  90,  90,   0,  400  }, // leg1,4 dn
  {    0, -45,  45,   0,   0,  45, -45,   0,  400  }, // protect myself
};

// Dancing 1
int servoPrg13step = 10;
int servoPrg13 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms                         ////check/////
  {   30,  90,  90, 150, 150,  90,  90,  30,  300  }, // leg1,2,3,4 up
  {  -30,   0,   0,   0,   0,   0,   0,   0,  300  }, // leg1 dn
  {   30,   0,   0,  30,   0,   0,   0,   0,  300  }, // leg1 up; leg2 dn
  {    0,   0,   0, -30,   0,   0,   0, -30,  300  }, // leg2 up; leg4 dn
  {    0,   0,   0,   0,  30,   0,   0,  30,  300  }, // leg4 up; leg3 dn
  {  -30,   0,   0,   0, -30,   0,   0,   0,  300  }, // leg3 up; leg1 dn
  {   30,   0,   0,  30,   0,   0,   0,   0,  300  }, // leg1 up; leg2 dn
  {    0,   0,   0, -30,   0,   0,   0, -30,  300  }, // leg2 up; leg4 dn
  {    0,   0,   0,   0,  30,   0,   0,  30,  300  }, // leg4 up; leg3 dn
  {    0,   0,   0,   0, -30,   0,   0,   0,  300  }, // leg3 up
};

// Dancing 2
int servoPrg14step = 9;
int servoPrg14 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms                                 ////check/////
  {   30,  45, 135, 150, 150, 135,  45,  30,  300  }, // leg1,2,3,4 two sides
  {   30,   0,   0, -30,   0,   0,   0,   0,  300  }, // leg1,2 up
  {  -30,   0,   0,  30, -30,   0,   0,  30,  300  }, // leg1,2 dn; leg3,4 up
  {   30,   0,   0, -30,  30,   0,   0, -30,  300  }, // leg3,4 dn; leg1,2 up
  {  -30,   0,   0,  30, -30,   0,   0,  30,  300  }, // leg1,2 dn; leg3,4 up
  {   30,   0,   0, -30,  30,   0,   0, -30,  300  }, // leg3,4 dn; leg1,2 up
  {  -30,   0,   0,  30, -30,   0,   0,  30,  300  }, // leg1,2 dn; leg3,4 up
  {   30,   0,   0, -30,  30,   0,   0, -30,  300  }, // leg3,4 dn; leg1,2 up
  {  -25,   0,   0,  25,   0,   0,   0,   0,  300  }, // leg1,2 dn
};

// Dancing 3
int servoPrg15step = 10;
int servoPrg15 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms                           ////check////
  {   30,  45,  38, 150, 150, 135, 147,  30,  300  }, // leg1,2,3,4 bk
  {   30,   0,   0, -40, -30,   0,   0,   0,  300  }, // leg1,2,3 up
  {  -30,   0,   0,  40,  30,   0,   0,   0,  300  }, // leg1,2,3 dn
  {   30,   0,   0,   0, -30,   0,   0,  40,  300  }, // leg1,3,4 up
  {  -30,   0,   0,   0,  30,   0,   0, -40,  300  }, // leg1,3,4 dn
  {   30,   0,   0, -40, -30,   0,   0,   0,  300  }, // leg1,2,3 up
  {  -30,   0,   0,  40,  30,   0,   0,   0,  300  }, // leg1,2,3 dn
  {   30,   0,   0,   0, -30,   0,   0,  40,  300  }, // leg1,3,4 up
  {  -30,   0,   0,   0,  30,   0,   0, -40,  300  }, // leg1,3,4 dn
  {    0,  45,  45,   0,   0, -45, -45,   0,  300  }, // standby
};

///////////////////////////////////////////////////////////function///////////////////////////////////////////////////////////

//runServoPrg
void runServoPrg(int servoPrg[][numberOfACE], int step)
{
  for (int i = 0; i < step; i++) { // Loop for step

    int totalTime = servoPrg[i][numberOfACE - 1]; // Total time of this step

    // Get servo start position
    for (int s = 0; s < numberOfServos; s++) {
      servoPos[s] = servo[s].read() - servoCal[s];
    }

    for (int j = 0; j < totalTime / servoPrgPeriod; j++) { // Loop for time section
      for (int k = 0; k < numberOfServos; k++) { // Loop for servo
        servo[k].write((map(j, 0, totalTime / servoPrgPeriod, servoPos[k], servoPrg[i][k])) + servoCal[k]);
      }
      delay(servoPrgPeriod);
    }
  }
}

// runServoPrg vector mode
void runServoPrgV(int servoPrg[][numberOfACE], int step) {
  for (int i = 0; i < step; i++) { // Loop for step

    int totalTime = servoPrg[i][numberOfACE - 1]; // Total time of this step

    // Get servo start position
    for (int s = 0; s < numberOfServos; s++) {
      servoPos[s] = servo[s].read() - servoCal[s];
    }

    for (int p = 0; p < numberOfServos; p++) { // Loop for servo
      if (i == 0) {
        servoPrevPrg[p] = servoPrg[i][p];
      } else {
        servoPrevPrg[p] = servoPrevPrg[p] + servoPrg[i][p];
      }
    }

    for (int j = 0; j < totalTime / servoPrgPeriod; j++) { // Loop for time section
      for (int k = 0; k < numberOfServos; k++) { // Loop for servo
        servo[k].write((map(j, 0, totalTime / servoPrgPeriod, servoPos[k], servoPrevPrg[k]) + servoCal[k]));
      }
      delay(servoPrgPeriod);
    }
  }
}

//check sensor
int sensor() {
  if (timeElapsed >= interval) {
    timeElapsed = 0; 

    // Perform the sensor reading
    uint16_t distance = VL53L0X_sensor.readRangeSingleMillimeters();

    // Check if an object is detected within the specified range
    if (distance > 0 && distance < MAX_DISTANCE) {
      // Object detected, set the flag to 1
      flag = 1;
    } else {
      // No object detected, set the flag to 0
      flag = 0;
    }

    // Print the distance and flag status
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" mm, Flag: ");
    Serial.println(flag); 
  }
  return flag;
}

//sensorSetup
void sensorSetup() {
  // Initialize the sensor
  if(!VL53L0X_sensor.init(0x29))
  {
    Serial.println("Failed to detect and initialize sensor!");
  }
  Serial.println("VL53L0X sensor detected!");
  VL53L0X_sensor.setTimeout(500);
}

//mpuSetup
void mpuSetup() {

  mpu.initialize(); // initialize MPU6050
  if (!mpu.testConnection()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(1047);
  mpu.setYAccelOffset(873);
  mpu.setZAccelOffset(1369);
  mpu.setXGyroOffset(133);
  mpu.setYGyroOffset(-86);
  mpu.setZGyroOffset(-12);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(6);
    //mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

//servoSetup
void servoSetup() {
  // Servo Pin Set
  servo[0].attach(0);
  servo[1].attach(1);
  servo[2].attach(2);
  servo[3].attach(3);
  servo[4].attach(6);
  servo[5].attach(7);
  servo[6].attach(8);
  servo[7].attach(9);
  
  servo[0].write(90 + servoCal[0]);
  servo[1].write(90 + servoCal[1]);
  servo[2].write(90 + servoCal[2]);
  servo[3].write(90 + servoCal[3]);
  servo[4].write(90 + servoCal[4]);
  servo[5].write(90 + servoCal[5]);
  servo[6].write(90 + servoCal[6]);
  servo[7].write(90 + servoCal[7]);
}

//PIDSetup
void PIDSetup(){
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-20, 20); //set the output limits
  myPID.SetSampleTime(10); //refresh rate
  myPID.SetTunings(Kp, Ki, Kd); //set PID gains
  Setpoint = 0; //setpoint
}

void MoveUpdateIn(){
  aux = Output;
  Forward[4][1] = Forward[4][1] + aux;
  Forward[6][1] = Forward[6][1] - aux;
  Forward[4][2] = Forward[4][2] - aux;
  Forward[7][2] = Forward[7][2] + aux;
  Forward[7][5] = Forward[7][5] + aux;
  Forward[10][5] = Forward[10][5] - aux;
  Forward[1][6] = Forward[1][6] - aux;
  Forward[4][6] = Forward[4][6] + aux;
}

void MoveUpdateOut(){
  Forward[4][1] = Forward[4][1] - aux;
  Forward[6][1] = Forward[6][1] + aux;
  Forward[4][2] = Forward[4][2] + aux;
  Forward[7][2] = Forward[7][2] - aux;
  Forward[7][5] = Forward[7][5] - aux;
  Forward[10][5] = Forward[10][5] + aux;
  Forward[1][6] = Forward[1][6] + aux;
  Forward[4][6] = Forward[4][6] - aux;
}

///////////////////////////////////////////////////////////setup///////////////////////////////////////////////////////////

//Setup
void setup() {

  Wire.begin(); // join i2c bus
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200);
  delay(1000);
  Serial.println("Serial Monitor Started");

  sensorSetup(); //sensor setup

  mpuSetup(); //mpu setup

  servoSetup(); //servo setup

  PIDSetup(); //PID setup

  delay(2000);

  runServoPrg(Zero, ZeroStep); // zero position

  delay(2000);
}

///////////////////////////////////////////////////////////loop///////////////////////////////////////////////////////////

//Loop
void loop() {

  mpu.dmpGetCurrentFIFOPacket(fifoBuffer); // read a packet from FIFO
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  Input = ypr[0] * 180/M_PI;

    //mpu.dmpGetQuaternion(&q, fifoBuffer);
    //mpu.dmpGetEuler(euler, &q);
    //Serial.print("euler_angle\t");
    //Serial.println(euler[0] * 180 / M_PI);
  
  Serial.print("Input: ");
  Serial.println(Input);
  Serial.print("Output: ");
  Serial.println(Output);
  Serial.print("Setpoint: ");
  Serial.println(Setpoint);

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  switch (currentState) {
    case Front:

      Setpoint = 0;
      myPID.Compute(); //compute PID

      MoveUpdateIn();
      runServoPrgV(Forward, ForwardStep); //move forward
      MoveUpdateOut();

      if(sensor() == 1 && side == 0){
        runServoPrgV(Backward, BackwardStep); //move backward
        for(int i=0; i<5; i++){
          runServoPrgV(servoPrg07, servoPrg07step); //turn right
        }
        currentState = Right;
      }

      if(sensor() == 1 && side == 1){
        runServoPrgV(Backward, BackwardStep); //move backward
        for(int i=0; i<5; i++){
          runServoPrgV(servoPrg06, servoPrg06step); //turn left
        }
        currentState = Left;
      }

      break;

    case Right:

      Setpoint = 90;
      myPID.Compute(); //compute PID

      MoveUpdateIn();
      runServoPrgV(Forward, ForwardStep); //move forward
      MoveUpdateOut();

      if(sensor() == 1){
        runServoPrgV(Backward, BackwardStep); //move backward
        for(int i=0; i<5; i++){
          //runServoPrgV(servoPrg06, servoPrg06step); //turn left
          runServoPrgV(servoPrg07, servoPrg07step); //turn right
        }
        //side = 1;
        currentState = Back;
      }
      break;

    case Left:

      Setpoint = -90;
      myPID.Compute(); //compute PID

      MoveUpdateIn();
      runServoPrgV(Forward, ForwardStep); //move forward
      MoveUpdateOut();

      if(sensor() == 1){
        runServoPrgV(Backward, BackwardStep); //move backward
        for(int i=0; i<5; i++){
          runServoPrgV(servoPrg07, servoPrg07step); //turn right
        }
        //side = 0;
        currentState = Front;
      }
      break;

    case Back:

      if(ypr[0] * 180/M_PI > 0 && ypr[0] * 180/M_PI < 180){
        Setpoint = 180;
      }
      else if(ypr[0] * 180/M_PI < 0 && ypr[0] * 180/M_PI > -180){
        Setpoint = -180;
      }

      myPID.Compute(); //compute PID

      MoveUpdateIn();
      runServoPrgV(Forward, ForwardStep); //move forward
      MoveUpdateOut();

      if(sensor() == 1){
        runServoPrgV(Backward, BackwardStep); //move backward
        for(int i=0; i<5; i++){
          runServoPrgV(servoPrg07, servoPrg07step); //turn right
        }
        //side = 0;
        currentState = Left;
      }
      break;
        
  }  
} //end of loop