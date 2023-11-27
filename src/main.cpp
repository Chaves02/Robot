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
#include <NewPing.h>
#include <elapsedMillis.h>

// Define HC-SR04 sensor pins
#define TRIGGER_PIN  17  // Arduino pin for trigger
#define ECHO_PIN     16  // Arduino pin for echo
#define MAX_DISTANCE 30 // Maximum distance in centimeters

// Create an instance of the NewPing library
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

elapsedMillis timeElapsed; 
int flag = 0; // Flag for checking if an object is detected
const long interval = 500;  // Interval for checking the sensor (in milliseconds)

const int numberOfServos = 8; // Number of servos
const int numberOfACE = 9; // Number of action code elements
int servoCal[] = { -2, -4, 2, -7, -5, 0, 0, 5 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoPrevPrg[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo previous prg
int servoPrgPeriod = 20; // 50 ms
Servo servo[numberOfServos]; // Servo object

// Zero
int servoPrg00step = 1;
int servoPrg00 [][numberOfACE]  = {
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
int servoPrg02step = 11;
int servoPrg02 [][numberOfACE]  = {
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
int servoPrg03step = 11;
int servoPrg03 [][numberOfACE]  = {
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
int servoPrg04step = 11;
int servoPrg04 [][numberOfACE]  = {
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
int servoPrg05step = 11;
int servoPrg05 [][numberOfACE]  = {
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

void sensor() {
  if (timeElapsed >= interval) {
    timeElapsed = 0; 

    // Perform the sensor reading
    int distance = sonar.ping_cm();

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
    Serial.print(" cm, Flag: ");
    Serial.println(flag);

  }
}

void setup() {

  Serial.begin(9600);
  // Servo Pin Set
  servo[0].attach(0);
  servo[1].attach(1);
  servo[2].attach(2);
  servo[3].attach(3);
  servo[4].attach(4);
  servo[5].attach(5);
  servo[6].attach(6);
  servo[7].attach(7);
  
  servo[0].write(90 + servoCal[0]);
  servo[1].write(90 + servoCal[1]);
  servo[2].write(90 + servoCal[2]);
  servo[3].write(90 + servoCal[3]);
  servo[4].write(90 + servoCal[4]);
  servo[5].write(90 + servoCal[5]);
  servo[6].write(90 + servoCal[6]);
  servo[7].write(90 + servoCal[7]);

  delay(2000);

  runServoPrg(servoPrg00, servoPrg00step); // zero position

  delay(2000);

}

void loop() {

  sensor();

  if (flag == 0)
    runServoPrgV(servoPrg02, servoPrg02step); //move forward
  if (flag == 1){
    runServoPrgV(servoPrg03, servoPrg03step); //move backward
    while(flag == 1){
      runServoPrgV(servoPrg07, servoPrg07step); //turn right
      sensor();
    }
  }
    
    
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg02, servoPrg02step); //move forward
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg03, servoPrg03step); //move backward
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg04, servoPrg04step); //move left
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg05, servoPrg05step); //move right
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg06, servoPrg06step); //turn left
  //}
  //
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg07, servoPrg07step); //turn right
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg08, servoPrg08step); //lie
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg09, servoPrg09step); //say hi
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg10, servoPrg10step); //fighting
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg11, servoPrg11step); //push up
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg12, servoPrg12step); //sleep
  //}
//
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg13, servoPrg13step); //dancing 1
  //}
  //for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg14, servoPrg14step); //dancing 2
  //}
  // for(int i=0; i<5; i++){
  //  runServoPrgV(servoPrg15, servoPrg15step); //dancing 3
  //}
  //
  //for(int i=0; i<15; i++){
  //  runServoPrgV(servoPrg01, servoPrg01step); //stand-by
  //}

  
}