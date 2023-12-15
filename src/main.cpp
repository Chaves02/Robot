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

#define MAX_DISTANCE 135 // Maximum distance in milimeters

VL53L0X VL53L0X_sensor; // VL53L0X object
elapsedMillis timeElapsed; // Time elapsed since the last measurement
const long interval = 500;  // Interval for checking the sensor (in milliseconds)

MPU6050 mpu(0x68); // MPU6050 object
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// MPU orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// PID direction controller
double Setpoint, directionAngle, Output;
const double Kp=0.8, Ki=5, Kd=0;
PID myPID(&directionAngle, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// PID climb controller
double Setpoint2, climbAngle, Output2;
const double Kp2=1.4, Ki2=0.5, Kd2=0;
PID myPID2(&climbAngle, &Output2, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

typedef enum{ // State machine states
  Front,
  Right,
  Left,
  Stair
} state;

state currentState = Front; // Initial state
int side = 0; // last side
int aux_direction = 0; // auxiliar variable for direction
int aux_climb = 0; // auxiliar variable for climb

const int numberOfServos = 8; // Number of servos
const int numberOfACE = 9; // Number of action code elements
int servoCal[] = { -3, -4, 2, -7, -5, 0, 0, 7 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoPrevPrg[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo previous prg
int servoPrgPeriod = 20; // 50 ms
Servo servo[numberOfServos]; // Servo object

///////////////////////////// Robot Actions /////////////////////////////

// Zero
int ZeroStep = 1;
int Zero [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30, 1000  }, // zero position
};

// Standby
int StandbySet = 2;
int Standby [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  200  }, // prep standby
  {  -20,   0,   0,  20,  20,   0,   0, -20,  200  }, // standby
};

// Check up
int CheckupStep = 2;
int Checkup [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   15,  90,  90, 165, 165,  90,  90,  15,  200  }, // standby
  {    0,   0,   0, -70,   0,   0,   0,  70,  400  }, // leg2,4 up
};

// Forward
int ForwardStep = 11;
int Forward [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby
  {   20,   0,   0,   0,   0,   0, -45,  20,  100  }, // leg1,4 up; leg4 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0, -45,  45,   0,   0,   0,  45,   0,  100  }, // leg1,4 bk; leg2 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,  45,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up; leg1 fw
  {    0,   0, -45,   0,   0,  45,   0,   0,  100  }, // leg2,3 bk
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0,   0, -20,   0,   0,   0,  100  }, // leg3 up
  {    0,   0,   0,   0,  20, -45,   0,   0,  100  }, // leg3 fw dn
};

// Backward
int BackwardStep = 11;
int Backward [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby
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

// Climb
int ClimbStep = 33;
int Climb [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   15,  90,  90, 165, 165,  90,  90,  15,  200  }, // standby
  {   20,   0,   0,   0,   0,   0, -50,  20,  200  }, // leg1,4 up; leg4 fw ///////////
  {  -20,   0,   0,   0,   0,   0,   0, -20,  200  }, // leg1,4 dn
  {    0,   0,   0, -20, -50,   0,   0,   0,  200  }, // leg2,3 up (gp4- 50)
  {    0, -50,  50,   0,   0,   0,  50,   0,  200  }, // leg1,4 bk; leg2 fw
  {    0,   0,   0,  20,  50,   0,   0,   0,  200  }, // leg2,3 dn
  {   50,  50,   0,   0,   0,   0,   0,  20,  200  }, // leg1,4 up; leg1 fw
  {    0,   0, -50,   0,   0,  50,   0,   0,  200  }, // leg2,3 bk
  {  -50,   0,   0,   0,   0,   0,   0, -20,  200  }, // leg1,4 dn
  {    0,   0,   0,   0, -20,   0,   0,   0,  200  }, // leg3 up
  {    0,   0,   0,   0,  20, -50,   0,   0,  200  }, // leg3 fw dn         ///////////
  {   20,   0,   0,   0, -20,   0,   0,   0,  200  }, // leg1,3 up   //baixa frente
  {   40,   0,   0,   0,   0,   0, -50,  20,  200  }, // leg1,4 up; leg4 fw ///////////
  {  -40,   0,   0,   0,   0,   0,   0, -20,  200  }, // leg1,4 dn
  {    0,   0,   0, -20, -40,   0,   0,   0,  200  }, // leg2,3 up
  {    0, -50,  50,   0,   0,   0,  70,   0,  200  }, // leg1,4 bk; leg2 fw
  {    0,   0,   0,  20,  40,   0,   0,   0,  200  }, // leg2,3 dn
  {   20,  50,   0,   0,   0,   0,   0,  20,  200  }, // leg1,4 up; leg1 fw 
  {    0,   0, -70,   0,   0,  50,   0,   0,  200  }, // leg2,3 bk
  {  -20,   0,   0,   0,   0,   0,   0, -20,  200  }, // leg1,4 dn
  {    0,   0,   0,   0,   0,   0,   0,  50,  200  }, // leg4 up     //começa patas de trás
  {    0,   0,   0,   0,   0,   0, -65,   0,  200  }, // leg4 fw
  {    0,   0,   0,   0,   0,   0,   0, -50,  200  }, // leg4 dn
  {    0,   0,   0,   0,   0, -70,   0,   0,  200  }, // leg3 fw 
  {    0,   0,   0,   0,   0,  70,  65,   0,  200  }, // leg3,4 bk
  {   10,   0,   0,   0, -10,   0,   0,   0,  200  }, // leg1,3 up   //baixa frente
  {    0,   0,   0, -50,   0,   0,   0,   0,  200  }, // leg2 up
  {    0,   0,  65,   0,   0,   0,   0,   0,  200  }, // leg2 fw
  {    0,   0,   0,  50,   0,   0,   0,   0,  200  }, // leg2 dn
  {    0, -70, -65,   0,   0,   0,   0,   0,  200  }, // leg1,2 bk
  {   60,   0,   0,   0, -60,   0,   0,   0,  200  }, // leg1,3 up   //baixa tudo frente
  {    0,  70,   0,   0,   0, -70,   0,   0,  200  },  //leg1,3 fw  //reset
  {  -90,   0,   0,   0,  90,   0,   0,   0,  200  }, // leg1,3 dn  //sobe frente
};

// Move Left
int MoveleftStep = 11;
int Moveleft [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby
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
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby
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
int TurnleftStep = 8;
int Turnleft [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0,  45,   0,   0,   0,   0,  45,   0,  100  }, // leg1,4 turn
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0,   0,  45,   0,   0,  45,   0,   0,  100  }, // leg2,3 turn
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {    0, -45, -45,   0,   0, -45, -45,   0,  100  }, // leg1,2,3,4 turn
};

// Turn right
int TurnrightStep = 8;
int Turnright [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  90,  90,  30,  100  }, // standby
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0,   0, -45,   0,   0, -45,   0,   0,  100  }, // leg2,3 turn
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0, -45,   0,   0,   0,   0, -45,   0,  100  }, // leg1,4 turn
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,  45,  45,   0,   0,  45,  45,   0,  100  }, // leg1,2,3,4 turn
};

// Lie
int LieStep = 6;
int Lie [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   30,  90,  90, 150, 150,  80,  90,  30, 100 }, //standby
  {  150,   0,   0, -20,   0,   0,   0,  20, 200 }, //leg1 maxup and leg2,4 up
  {    0,  45,   0,   0,   0,   0,   0,   0, 350 }, //leg1 fw
  {    0, -45,   0,   0,   0,   0,   0,   0, 350 }, //leg1 bk
  {    0,  45,   0,   0,   0,   0,   0,   0, 350 }, //leg1 fw
  {    0, -45,   0,   0,   0,   0,   0,   0, 350 }  //leg1 bk
};

// Say Hi
int SayhiStep = 4;
int Sayhi [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {    0,  90,  90, 150, 180,  90,  90,  30,  200  }, // leg1, 3 down
  {   30,   0,   0,   0, -30,   0,   0,   0,  200  }, // standby
  {  -30,   0,   0,   0,  30,   0,   0,   0,  200  }, // leg1, 3 down
  {   30,   0,   0,   0, -30,   0,   0,   0,  200  }, // standby
};

// Fighting
int FightingStep = 11;
int Fighting [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
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
int PushupStep = 11;
int Pushup [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
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
int SleepStep = 2;
int Sleep [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {    0,  90,  90, 150, 150,  90,  90,   0,  400  }, // leg1,4 dn
  {    0, -45,  45,   0,   0,  45, -45,   0,  400  }, // protect myself
};

// Dancing 1
int Dance1Step = 10;
int Dance1 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
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
int Dance2Step = 9;
int Dance2 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
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
int Dance3Step = 10;
int Dance3 [][numberOfACE]  = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
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

/////////////////////////////  Functions  /////////////////////////////////

// runServoPrg
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

// check sensor
int sensor() {
  int flag = 0; // Flag to indicate if an object is detected
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

// sensorSetup
void sensorSetup() {
  // Initialize the sensor
  if(!VL53L0X_sensor.init(0x29))
  {
    Serial.println("Failed to detect and initialize sensor!");
  }
  Serial.println("VL53L0X sensor detected!");
  VL53L0X_sensor.setTimeout(500);
}

// mpuSetup
void mpuSetup() {

  mpu.initialize(); // initialize MPU6050
  if (!mpu.testConnection()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  Serial.println("MPU6050 Found!");

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, uncomment if you want to manually set offsets
  //mpu.setXAccelOffset(1047);
  //mpu.setYAccelOffset(873);
  //mpu.setZAccelOffset(1369);
  //mpu.setXGyroOffset(133);
  //mpu.setYGyroOffset(-86);
  //mpu.setZGyroOffset(-12);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(20);
    mpu.CalibrateGyro(20);
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

// servoSetup
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

// PID Direction Setup
void PIDSetup(){
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-30, 30); //set the output limits
  myPID.SetSampleTime(10); //refresh rate
  myPID.SetTunings(Kp, Ki, Kd); //set PID gains
  Setpoint = 0; //setpoint
}

// PID Climb Setup
void PID2Setup(){
  //turn the PID on
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-20, 20); //set the output limits
  myPID2.SetSampleTime(10); //refresh rate
  myPID2.SetTunings(Kp2, Ki2, Kd2); //set PID gains
  Setpoint2 = Setpoint; //setpoint
}

// Direction Update
void MoveUpdate(){
  aux_direction = Output;
  Forward[4][1] = -45 + aux_direction;
  Forward[6][1] =  45 - aux_direction;
  Forward[4][2] =  45 - aux_direction;
  Forward[7][2] = -45 + aux_direction;
  Forward[7][5] =  45 + aux_direction;
  Forward[10][5]= -45 - aux_direction;
  Forward[1][6] = -45 - aux_direction;
  Forward[4][6] =  45 + aux_direction;
}

// Climb Update
void ClimbUpdate(){
  aux_climb = Output2;
  Forward[0][0] = 30 + aux_climb;
  Forward[0][3] = 150 + aux_climb;
  Forward[0][4] = 150 - aux_climb;
  Forward[0][7] = 30 - aux_climb;
}

// read mpu values
void mpuGetValues(){
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer); // read a packet from FIFO
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  directionAngle = ypr[0] * 180/M_PI;
  climbAngle = ypr[2] * 180/M_PI;
}

/////////////////////////////  Setup  /////////////////////////////////
void setup() {

  Wire.begin(); // join i2c bus
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(115200); // initialize serial communication
  
  servoSetup(); //servo setup

  delay(2000); //wait for 3 seconds

  runServoPrg(Zero, ZeroStep); // zero position
  
  delay(2000);
  
  sensorSetup(); //sensor setup

  mpuSetup(); //mpu setup

  PIDSetup(); //PID setup

  PID2Setup(); //PID2 setup
}

/////////////////////////////  Loop  /////////////////////////////////

void loop() {

  mpuGetValues(); //get values from mpu
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  switch (currentState) {

    case Front:

      Setpoint = 0;
      myPID.Compute(); //compute PID
      myPID2.Compute(); //compute PID2
      
      ClimbUpdate(); //update climb
      MoveUpdate(); //update move
      runServoPrgV(Forward, ForwardStep); //move forward

      mpuGetValues(); //get values from mpu
      //Serial.print("Climb angle : ");
      //Serial.println(climbAngle);
      if((sensor() == 1 && climbAngle < 2)){
        runServoPrgV(Checkup, CheckupStep); //checkup
        if(sensor() == 0){
          currentState = Stair;
        }
        else {
          if(side == 0){
            runServoPrgV(Backward, BackwardStep); //move backward
            while(directionAngle < 75){
              runServoPrgV(Turnright, TurnrightStep); //turn right
              mpuGetValues(); //get values from mpu
            }
            currentState = Right;
          }
          if(side == 1){
            runServoPrgV(Backward, BackwardStep); //move backward
            while(directionAngle > -75){
              runServoPrgV(Turnleft, TurnleftStep); //turn left
              mpuGetValues(); //get values from mpu
            }
            currentState = Left;
          }        
        }
      }

    break;

    case Right:

      Setpoint = 90;
      myPID.Compute(); //compute PID
      myPID2.Compute(); //compute PID2
      ClimbUpdate(); //update climb

      for(int i=0; i<7; i++){
        if(sensor() == 0){
        mpuGetValues(); //get values from mpu
        myPID.Compute(); //compute PID
        MoveUpdate(); //update move
        runServoPrgV(Forward, ForwardStep); //move forward
        }
        else{
          break;
        }
      }

      while(directionAngle > 15 || directionAngle < -15){
        runServoPrgV(Turnleft, TurnleftStep); //turn left
        mpuGetValues(); //get values from mpu
        side = 1;
      }     
      currentState = Front;

    break;

    case Left:

      Setpoint = -90;
      myPID.Compute(); //compute PID
      myPID2.Compute(); //compute PID2
      ClimbUpdate(); //update climb

      for(int i=0; i<7; i++){
        if(sensor() == 0){
        mpuGetValues(); //get values from mpu
        myPID.Compute(); //compute PID
        MoveUpdate();
        runServoPrgV(Forward, ForwardStep); //move forward
        }
        else{
          break;
        }
      }

      while(directionAngle > 15 || directionAngle < -15){
        runServoPrgV(Turnright, TurnrightStep); //turn right
        mpuGetValues(); //get values from mpu
        side = 0;
      }

      currentState = Front;

    break;

   // case Back:
   //   if(ypr[0] * 180/M_PI > 0 && ypr[0] * 180/M_PI < 180){
   //     Setpoint = 180;
   //   }
   //   else if(ypr[0] * 180/M_PI < 0 && ypr[0] * 180/M_PI > -180){
   //     Setpoint = -180;
   //   }
   //   myPID.Compute(); //compute PID
   //   myPID2.Compute(); //compute PID2
   //   ClimbUpdate(); //update climb
   //   MoveUpdate();
   //   runServoPrgV(Forward, ForwardStep); //move forward
   //   if(sensor() == 1){
   //     runServoPrgV(Backward, BackwardStep); //move backward
   //     for(int i=0; i<5; i++){
   //       runServoPrgV(servoPrg07, servoPrg07step); //turn right
   //     }
   //     //side = 0;
   //     currentState = Left;
   //   }
   //   break;
     
    case Stair:

      mpuGetValues(); //get values from mpu
      myPID.Compute(); //compute PID

      MoveUpdate();
      runServoPrgV(Forward, ForwardStep); //move forward
      runServoPrgV(Climb, ClimbStep); //climb stair

      currentState = Front;

    break;  
  }  
}
/////////////////////////////  End  /////////////////////////////////