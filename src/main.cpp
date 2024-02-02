// Spider_mini (Top View)
//  -----               -----
// |  L3 |             |  L1 |
// | GP6 |             | GP0 |
//  ----- -----   ----- -----
//       |     | |     |
//       | GP7 | | GP1 |
//        -----   -----
//       |     | |     |
//       | GP8 | | GP2 |
//  ----- -----   ----- -----
// |  L4 |             |  L2 |
// | GP9 |             | GP3 |
//  -----               -----

#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "moves.h"

#define BUTTON_PIN 17 // Button pin

int MAX_DISTANCE = 135; // Maximum distance in milimeters
VL53L0X VL53L0X_sensor; // VL53L0X object
MPU6050 mpu(0x68); // MPU6050 object

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)+
uint8_t fifoBuffer[64]; // FIFO storage buffer
// MPU orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// PID direction controller
double Setpoint, directionAngle, Output;
const double Kp=0.8, Ki=5, Kd=0;
PID myPID(&directionAngle, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// PID climb controller
double Setpoint2, climbAngle, Climb_Output;
const double Kp2=1.4, Ki2=0.5, Kd2=0;
PID myPID2(&climbAngle, &Climb_Output, &Setpoint2, Kp2, Ki2, Kd2, DIRECT);

int side = 0; // last side

typedef enum{ // Main state machine
  Front,
  Right,
  Left,
  Clear,
  Obstacle,
  Stair
} state;
state currentState = Front; // Initial state

typedef enum{ // Mode select state machine
  Mode1,
  Mode2,
  Mode3,
  Mode4,
  Mode5,
} operation_mode;
operation_mode currentMode = Mode1; // Initial mode

const int numberOfServos = 8; // Number of servos
int servoCal[] = { -3, -4, 2, -7, -5, 0, 0, 7 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoPrevPrg[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo previous prg
int servoPrgPeriod = 10; // 10 ms
Servo servo[numberOfServos]; // Servo object

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
    mpu.CalibrateAccel(30);
    mpu.CalibrateGyro(30);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
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
  Setpoint = 0; //setpoint
}

// PID Climb Setup
void PID2Setup(){
  //turn the PID on
  myPID2.SetMode(AUTOMATIC);
  myPID2.SetOutputLimits(-20, 20); //set the output limits
  Setpoint2 = 0; //setpoint
}

// Front Update
void MoveFrontUpdate(){
  Forward[4][1] = -45 + Output;
  Forward[6][1] =  45 - Output;
  Forward[4][2] =  45 - Output;
  Forward[7][2] = -45 + Output;
  Forward[7][5] =  45 + Output;
  Forward[10][5]= -45 - Output;
  Forward[1][6] = -45 - Output;
  Forward[4][6] =  45 + Output;
}

// Left Update
void MoveLeftUpdate(){
  Moveleft[4][1] =   45 - Output;
  Moveleft[7][1] =  -45 + Output;
  Moveleft[1][2] =  -45 - Output;
  Moveleft[4][2] =   45 + Output;
  Moveleft[4][5] =  -45 + Output;
  Moveleft[6][5]=    45 - Output;
  Moveleft[7][6] =   45 + Output;
  Moveleft[10][6] = -45 - Output;
}

// Right Update
void MoveRightUpdate(){
  Moveright[7][1] =   50 + Output;
  Moveright[10][1] = -50 - Output;
  Moveright[4][2] =  -45 + Output;
  Moveright[6][2] =   45 - Output;
  Moveright[1][5] =  -50 - Output;
  Moveright[4][5]=    50 + Output;
  Moveright[4][6] =   45 - Output;
  Moveright[7][6] =  -45 + Output;
}

// Climb Update
void ClimbUpdate(){
  Forward[0][0] =  30 + Climb_Output;
  Forward[0][3] = 150 + Climb_Output;
  Forward[0][4] = 150 - Climb_Output;
  Forward[0][7] =  30 - Climb_Output;
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

//Main State Machine
void SpiderMini(){

  switch (currentState) {

    case Front:
      MAX_DISTANCE = 135;
      Setpoint = 0;
      ClimbUpdate(); //update climb
      MoveFrontUpdate(); //update move
      runServoPrgV(Forward, ForwardStep); //move forward
      //Serial.print("Climb angle : ");
      //Serial.println(climbAngle);
      if((sensor() == 1 && climbAngle < 2))
        currentState = Obstacle;
    break;
    
    case Obstacle:
      runServoPrgV(Checkup, CheckupStep); //checkup
      if(sensor() == 0){
          currentState = Stair;
      }
      else{
        if(side == 0)
          currentState = Right;
        if(side == 1)
          currentState = Left; 
      }     
    break;

    case Right:
      MAX_DISTANCE = 190;
      MoveRightUpdate();      
      runServoPrgV(Moveright, MoverightStep); //move right
      if(sensor() == 0){
        side = 1;
        currentState = Clear;
      }
    break;

    case Left:
      MAX_DISTANCE = 190;
      MoveLeftUpdate();
      runServoPrgV(Moveleft, MoveleftStep); //move left
      if(sensor() == 0){
        side = 0;
        currentState = Clear;
      }
    break;

    case Clear:
      for(int i=0; i<3; i++){
        mpuGetValues(); //get values from mpu
        myPID.Compute(); //compute PID
        if(side == 0){
          MoveLeftUpdate();
          runServoPrgV(Moveleft, MoveleftStep); //move left
        }          
        else if(side == 1){
          MoveRightUpdate();
          runServoPrgV(Moveright, MoverightStep); //move right
        }
      }
      currentState = Front;
    break;
    
    case Stair:
      MoveFrontUpdate();
      runServoPrgV(Forward, ForwardStep); //move forward
      runServoPrgV(Climb, ClimbStep); //climb stair
      currentState = Front;
    break;

  }
}

// Mode Select State Machine
void ModeSelect(){
   switch (currentMode){

    case Mode1: //normal mode
      SpiderMini();
      if (digitalRead(BUTTON_PIN) == LOW) {
        currentMode = Mode2;
        Serial.println("Mode 2");
        delay(1000);
      }
    break;

    case Mode2: //push up
      runServoPrgV(Pushup, PushupStep);
      runServoPrgV(Lie, LieStep); 
      if (digitalRead(BUTTON_PIN) == LOW) {
        currentMode = Mode3;
        Serial.println("Mode 3");
        delay(1000);
      }
    break;

    case Mode3: //dance 1
      runServoPrgV(Dance1, Dance1Step); 
      if (digitalRead(BUTTON_PIN) == LOW) {
        currentMode = Mode4;
        Serial.println("Mode 4");
        delay(1000);
      }
    break;

    case Mode4: //dance 2
      runServoPrgV(Dance2, Dance2Step);
      if (digitalRead(BUTTON_PIN) == LOW) {
        currentMode = Mode5;
        Serial.println("Mode 5");
        delay(1000);
      }
    break;

    case Mode5: //dance 3
      runServoPrgV(Dance3, Dance3Step);
      if (digitalRead(BUTTON_PIN) == LOW) {
        currentMode = Mode1;
        Serial.println("Mode 1");
        delay(1000);
      }
    break;

   }
}

/////////////////////////////  Setup  /////////////////////////////////

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP); //pin mode
  Wire.begin(); // join i2c bus
  Serial.begin(115200); // initialize serial communication
  servoSetup(); //servo setup
  delay(3000); //wait for servo setup
  runServoPrgV(Standby, StandbyStep); //standby position  
  sensorSetup(); //sensor setup
  mpuSetup(); //mpu setup
  PIDSetup(); //PID setup
  PID2Setup(); //PID2 setup
  runServoPrgV(Sayhi, SayhiStep); //say hi
  runServoPrg(Zero, ZeroStep); // zero position
}

/////////////////////////////  Loop  /////////////////////////////////

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  mpuGetValues(); //get values from mpu
  myPID.Compute(); //compute PID
  myPID2.Compute(); //compute PID2
  ModeSelect(); //mode select
}

/////////////////////////////  End  /////////////////////////////////