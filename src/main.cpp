// Spider_mini (Top View)
//  -----               -----
// |  4  |             |  0  |
// | GP4 |             | GP0 |
//  ----- -----   ----- -----
//       |  5  | |  1  |
//       | GP5 | | GP1 |
//        -----   -----
//       |  6  | |  2  |
//       | GP6 | | GP2 |
//  ----- -----   ----- -----
// |  7  |             |  3  |
// | GP7 |             | GP3 |
//  -----               -----

#include <Arduino.h>
#include <Servo.h>

const int numberOfServos = 8; // Number of servos
const int numberOfACE = 9; // Number of action code elements
int servoCal[] = { 0, 17, -10, 0, 0, 0, 1, 0 }; // Servo calibration data
int servoPos[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo current position
int servoPrevPrg[] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // Servo previous prg
int servoPrgPeriod = 20; // 50 ms
Servo servo[numberOfServos]; // Servo object

// Servo zero position
int servoAct00 [] PROGMEM =
// GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7
{  135,  45, 135,  45,  45, 135,  45, 135 };

// Zero
int servoPrg00step = 1;
int servoPrg00 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {  135,  45, 135,  45,  45, 135,  45, 135, 1000  }, // zero position
};

// Standby
int servoPrg01step = 2;
int servoPrg01 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  200  }, // prep standby
  {  -20,   0,   0,  20,  20,   0,   0, -20,  200  }, // standby
};

// Forward
int servoPrg02step = 11;
int servoPrg02 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
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
int servoPrg03step = 11;
int servoPrg03 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {   20, -45,   0,   0,   0,   0,   0,  20,  100  }, // leg4,1 up; leg1 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg3,2 up
  {    0,  45,   0,   0,   0,  45, -45,   0,  100  }, // leg4,1 bk; leg3 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {   20,   0,   0,   0,   0,   0,  45,  20,  100  }, // leg4,1 up; leg4 fw
  {    0,   0,  45,   0,   0, -45,   0,   0,  100  }, // leg3,1 bk
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,   0, -20,   0,   0,   0,   0,  100  }, // leg2 up
  {    0,   0, -45,  20,   0,   0,   0,   0,  100  }, // leg2 fw dn
};

// Move Left
int servoPrg04step = 11;
int servoPrg04 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {    0,   0, -45, -20, -20,   0,   0,   0,  100  }, // leg3,2 up; leg2 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0,  45,  45,   0,   0, -45,   0,   0,  100  }, // leg3,2 bk; leg1 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,   0,   0, -20, -20,  45,   0,   0,  100  }, // leg3,2 up; leg3 fw
  {    0, -45,   0,   0,   0,   0,  45,   0,  100  }, // leg1,4 bk
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg3,2 dn
  {    0,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg4 up
  {    0,   0,   0,   0,   0,   0, -45, -20,  100  }, // leg4 fw dn
};

// Move Right
int servoPrg05step = 11;
int servoPrg05 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {    0,   0,   0, -20, -20, -45,   0,   0,  100  }, // leg2,3 up; leg3 fw
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg4,1 up
  {    0,   0, -45,   0,   0,  45,  45,   0,  100  }, // leg2,3 bk; leg4 fw
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg4,1 dn
  {    0,   0,  45, -20, -20,   0,   0,   0,  100  }, // leg2,3 up; leg2 fw
  {    0,  45,   0,   0,   0,   0, -45,   0,  100  }, // leg4,1 bk
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,   0,  100  }, // leg1 up
  {  -20, -45,   0,   0,   0,   0,   0,   0,  100  }, // leg1 fw dn
};

// Turn left
int servoPrg06step = 8;
int servoPrg06 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
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
int servoPrg07 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  100  }, // standby
  {    0,   0,   0, -20, -20,   0,   0,   0,  100  }, // leg2,3 up
  {    0,   0, -45,   0,   0, -45,   0,   0,  100  }, // leg2,3 turn
  {    0,   0,   0,  20,  20,   0,   0,   0,  100  }, // leg2,3 dn
  {   20,   0,   0,   0,   0,   0,   0,  20,  100  }, // leg1,4 up
  {    0, -45,   0,   0,   0,   0, -45,   0,  100  }, // leg1,4 turn
  {  -20,   0,   0,   0,   0,   0,   0, -20,  100  }, // leg1,4 dn
  {    0,  45,  45,   0,   0,  45,  45,   0,  100  }, // leg1,2,3,4 turn
};

// Lie
int servoPrg08step = 1;
int servoPrg08 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {  110,  90,  90,  70,  70,  90,  90, 110,  500  }, // leg1,4 up
};

// Say Hi
int servoPrg09step = 4;
int servoPrg09 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 3 down
  {  -50,   0,   0,   0,  50,   0,   0,   0,  200  }, // standby
  {   50,   0,   0,   0, -50,   0,   0,   0,  200  }, // leg1, 3 down
  {  -50,   0,   0,   0,  50,   0,   0,   0,  200  }, // standby
};

// Fighting
int servoPrg10step = 11;
int servoPrg10 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {  120,  90,  90, 110,  60,  90,  90,  70,  200  }, // leg1, 2 down
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -40, -40,   0,   0, -40, -40,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {  -50, -20, -20, -40,  50, -20, -20,  40,  200  }, // leg1, 2 up ; leg3, 4 down
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -40, -40,   0,   0, -40, -40,   0,  200  }, // body turn left
  {    0,  40,  40,   0,   0,  40,  40,   0,  200  }, // body turn right
  {    0, -20, -20,   0,   0, -20, -20,   0,  200  }, // leg1, 2 up ; leg3, 4 down
};

// Push up
int servoPrg11step = 11;
int servoPrg11 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  90,  90, 110, 110,  90,  90,  70,  300  }, // start
  {   30,   0,   0, -30, -30,   0,   0,  30,  400  }, // down
  {  -30,   0,   0,  30,  30,   0,   0, -30,  500  }, // up
  {   30,   0,   0, -30, -30,   0,   0,  30,  600  }, // down
  {  -30,   0,   0,  30,  30,   0,   0, -30,  700  }, // up
  {   30,   0,   0, -30, -30,   0,   0,  30,  1300 }, // down
  {  -30,   0,   0,  30,  30,   0,   0, -30,  1800 }, // up
  {   65,   0,   0, -65, -65,   0,   0,  65,  200  }, // fast down
  {  -65,   0,   0,   0,  15,   0,   0,   0,  500  }, // leg1 up
  {    0,   0,   0,   0,  50,   0,   0,   0,  500  }, // leg2 up
  {    0,   0,   0,  65,   0,   0,   0, -65,  500  }, // leg3, leg4 up
};

// Sleep
int servoPrg12step = 2;
int servoPrg12 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   35,  90,  90, 145, 145,  90,  90,  35,  400  }, // leg1,4 dn
  {    0, -45,  45,   0,   0,  45, -45,   0,  400  }, // protect myself
};

// Dancing 1
int servoPrg13step = 10;
int servoPrg13 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   90,  90,  90,  90,  90,  90,  90,  90,  300  }, // leg1,2,3,4 up
  {  -40,   0,   0,   0,   0,   0,   0,   0,  300  }, // leg1 dn
  {   40,   0,   0,  40,   0,   0,   0,   0,  300  }, // leg1 up; leg2 dn
  {    0,   0,   0, -40,   0,   0,   0, -40,  300  }, // leg2 up; leg4 dn
  {    0,   0,   0,   0,  40,   0,   0,  40,  300  }, // leg4 up; leg3 dn
  {  -40,   0,   0,   0, -40,   0,   0,   0,  300  }, // leg3 up; leg1 dn
  {   40,   0,   0,  40,   0,   0,   0,   0,  300  }, // leg1 up; leg2 dn
  {    0,   0,   0, -40,   0,   0,   0, -40,  300  }, // leg2 up; leg4 dn
  {    0,   0,   0,   0,  40,   0,   0,  40,  300  }, // leg4 up; leg3 dn
  {    0,   0,   0,   0, -40,   0,   0,   0,  300  }, // leg3 up
};

// Dancing 2
int servoPrg14step = 9;
int servoPrg14 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  45, 135, 110, 110, 135,  45,  70,  300  }, // leg1,2,3,4 two sides
  {   45,   0,   0, -45,   0,   0,   0,   0,  300  }, // leg1,2 up
  {  -45,   0,   0,  45, -45,   0,   0,  45,  300  }, // leg1,2 dn; leg3,4 up
  {   45,   0,   0, -45,  45,   0,   0, -45,  300  }, // leg3,4 dn; leg1,2 up
  {  -45,   0,   0,  45, -45,   0,   0,  45,  300  }, // leg1,2 dn; leg3,4 up
  {   45,   0,   0, -45,  45,   0,   0, -45,  300  }, // leg3,4 dn; leg1,2 up
  {  -45,   0,   0,  45, -45,   0,   0,  45,  300  }, // leg1,2 dn; leg3,4 up
  {   45,   0,   0, -45,  45,   0,   0, -45,  300  }, // leg3,4 dn; leg1,2 up
  {  -40,   0,   0,  40,   0,   0,   0,   0,  300  }, // leg1,2 dn
};

// Dancing 3
int servoPrg15step = 10;
int servoPrg15 [][numberOfACE] PROGMEM = {
  // GP0, GP1, GP2, GP3, GP4, GP5, GP6, GP7,  ms
  {   70,  45,  45, 110, 110, 135, 135,  70,  300  }, // leg1,2,3,4 bk
  {   40,   0,   0, -50, -40,   0,   0,   0,  300  }, // leg1,2,3 up
  {  -40,   0,   0,  50,  40,   0,   0,   0,  300  }, // leg1,2,3 dn
  {   40,   0,   0,   0, -40,   0,   0,  50,  300  }, // leg1,3,4 up
  {  -40,   0,   0,   0,  40,   0,   0, -50,  300  }, // leg1,3,4 dn
  {   40,   0,   0, -50, -40,   0,   0,   0,  300  }, // leg1,2,3 up
  {  -40,   0,   0,  50,  40,   0,   0,   0,  300  }, // leg1,2,3 dn
  {   40,   0,   0,   0, -40,   0,   0,  50,  300  }, // leg1,3,4 up
  {  -40,   0,   0,   0,  40,   0,   0, -50,  300  }, // leg1,3,4 dn
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

void setup() {
  
  
  //getServoCal(); // Get servoCal from EEPROM 

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
  servo[1].write(0 + servoCal[1]);
  servo[2].write(180 + servoCal[2]);
  servo[3].write(90 + servoCal[3]);
  servo[4].write(90 + servoCal[4]);
  servo[5].write(180 + servoCal[5]);
  servo[6].write(0 + servoCal[6]);
  servo[7].write(90 + servoCal[7]);

  //runServoPrg(servoPrg00, servoPrg00step); // zero position
  //servo[0].write(180+servoCal[0]);

}

void loop() {

  
  //servo[0].write(0);
  //runServoPrgV(servoPrg02, servoPrg02step); //move forward
  //delay(250);
  //runServoPrgV(servoPrg01, servoPrg01step); //stand-by
  //delay(250);


}

//-----------------------------------function--------------------------------------------------



// Get servoCal from EEPROM
void getServoCal() {
  int eeAddress = 0;
  for (int i = 0; i < numberOfServos; i++) {
    //EEPROM.get(eeAddress, servoCal[i]);
    eeAddress += sizeof(servoCal[i]);
  }
}

// Put servoCal to EEPROM
void putServoCal() {
  int eeAddress = 0;
  for (int i = 0; i < numberOfServos; i++) {
    //EEPROM.put(eeAddress, servoCal[i]);
    eeAddress += sizeof(servoCal[i]);
  }
}

// Clear Servo calibration data
void clearCal() {
  for (int i = 0; i < numberOfServos; i++) {
    servoCal[i] = 0;
  }
  putServoCal(); // Put servoCal to EEPROM
  runServoPrg(servoPrg00, servoPrg00step); // zero position
}

// Calibration
void calibration(int i, int change) {
  servoCal[i] = servoCal[i] + change;
  servo[i].write(servoAct00[i] + servoCal[i]);
  putServoCal(); // Put servoCal to EEPROM
  delay(400);
}