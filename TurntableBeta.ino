//TurntableBeta, based on TurntableNB12 By Steve Lomax 22/11/2024 not for commercial purposes. 
// this is, and will always remain, a work in progress. This is a working version, 
/*
Features:
  10 preset positions on 5 buttons (long/short press). 
  this will permit either end of the turntable bridge to face each of 5 track rays
  movement may be controlled by manual push switch and/or from JMRI over C/MRI. JMRI C/MRI  is not essential.
  Manual rotation and position setting (calibrating) is controlled by the encoder
  To offset any backlash and assist in alignment the final approach to each position will always be clockwise.
  (an anti-clockwise move will overshoot a few degrees  and creep back clockwise)
  Turntable  movements have acceleration and deceleration curves settable prior to upload
  The Turntable will take shortest route to the target position from the current position
  The target position may be changed mid rotation from the panel switches JMRI control is only possible when the turntable is stationary.
  the turntable will re-set the zero position each time it passes the zero position and each time the calibration (encoder long push) button is pressed 
  When in calibration mode, pressing any button (short or long press) will record the current position into that button memory overwriting the previous setting.

  all switches and encoder connect pin to ground
  all LEDs connect pin to ground via limiting resistor (typ 200R)
  microswitch fitted to turntable to mark 'position 0' which may be anywhere,
  but ideally somewhere between the last and first track ray.
  
  Hardware:
  Turntable
  Stepper motor 28BYJ-48 and ULN2003 driver module
  Micro switch positioned to trigger when the turntable is at 'home' position (zero) Idelly this would be anticlockwise before position 1
  The software will allow for switch dwell angle so cam operation is possible.
  6 push buttons (5 rays and reset)
  rotary Encoder module (pullup resistors and debounce capacitor fitted)
  LED with series resistor for 5V
  5V 1 amp power supply
  Arduino
  Connecting leads
  Mounting panel




  CONNECTIONS
  pin 0 DI on 485 Module
  pin 1 RO on 485 Module
  pin 2 encoder data 1
  pin 3 encoder data 2
  pin 4 DE/RE (joined)  on 485 Module
  Pin 5 encoder push button
    Pins 6,7,8,9 to stepper 1,2,3,4
  pin 10 limit (position 0) microswitch
  Pin 13 LED via resistor
  Pin 14 (A0) move/store to ray 1
  Pin 15 (A1) move/store to ray 2
  Pin 16 (A2) move/store to ray 3
  Pin 17 (A3) move/store to ray 4
  Pin 12      move/store to ray 5
  pin 11 for LED data in a future version

  OPERATION
  
  pressing any one of 5 position buttons will align the turntable to a ray track associated with that button
  Long pressing any one of the buttons will align the turntable to the track associated with a long press of that button.
  The intention here is that a long press aligns the table with that track in reverse,
  but this need not necessarily be so. Any position can be saved under a long press.
  Long pressing the encoder button will enter calibration mode.
  Short pressing the calibration button will exit calibration mode
  While in calibration mode, Short pressing a ray button will store the cutrrent table position to that button
  Long pressing a ray position will store the position associated with a long press of that button.
  
  

  on first power-up there will be no saved positions so the device will appear not to work.
  Initial calibration will need to be carried out on first power up.

  CALIBRATION MODE
  Press in the encoder button for 2 seconds.
  the turntable will home to default position 0
  the encoder may be turned and the turntable will move accordingly
  up to 10 positions can be saved by either a short or long press of the 5 position buttons.
  Short press the encoder button to exit calibration

  NORMAL MODE
  Long or short pressing a ray button will move the turntable to the stored position
  Manual operation can be carried out in calibration mode.
  
  JMRI control will require CMRI node 1 to be set for 10 lights or outputs and 10 sensors or inputs. 
  Placing 10 sensors on the layout image and attaching these trigger to a light or turnout will send the turntable to that position 
  Placing a further 10 sensors on the layout image will show as active for the current position.    

  TO RESET
  The turntable will reset to position 0 (home) each time the power is switched on and
  each time calibration mode is entered.

  LIMITATION
  Always calibrate in the same direction as the turntable moves when homing.
  Positions greater than 2 full turns (720 degrees) from home will not be correctly stored.
  the turntable will lose its position and  must be reset if movement is prevented or restricted.

*/


// if JMRI is not being used and only manual control is required delete or comment out lines following //FOR JMRI 


#include <Encoder.h> // by paul Stoffregen 1.4.4 
#include <EEPROM.h> // internal
#include <LibPrintf.h> // by Embedded Artistry in the Arduino library 
#include <AccelStepper.h> // by Mike McCauley in the library manager or from https://github.com/waspinator/AccelStepper

//next 4 lines FOR JMRI 
#include <CMRI.h>    // by Michael D K Adams from https://github.com/madleech/ArduinoCMRI
#include <Auto485.h> // by Michael D K Adams from https://github.com/madleech/Auto485
#define CMRI_ADDR 1  // This will be card 1
#define DE_PIN 4        //for Auto485.h

#define DATA_PIN 11     //LED data pin
#define ROT_ENCA_PIN 3  // Rotary encoder A - Pin supports interrupts
#define ROT_ENCB_PIN 2  // Rotary encoder B - Pin supports interrupts
#define ZERO_ON_THE_FLY_LOW_LIMIT 100
#define ZERO_ON_THE_FLY_HIGH_LIMIT 150

bool DEBUG = 1;         //set to 0 to cancel output text to console
const int ACCEL = 5;    //reduce for slower accel and decel
const int SPEED = 600;  //

const int ROTATION_STEPS = 2048;  //2038;//2116;
const int HALF_ROTATION_STEPS = ROTATION_STEPS / 2;
const byte MICROSW = 10;
const int CCW_OVERSHOOT = 50;
unsigned long buttonTimeout;
const long buttonDelay = 1000;
int movPos;
int but[6] = { 5, 14, 15, 16, 17, 12 };  // Position Buttons (0 is encoder -> calibrate)
int pos[11]; // 10 ray positions and zero (home) position
const int LED = 13; // calibrating / moving
int newButton;// buttom last pressed
int oldButton;//button previously pressed
int newPos;// new target
int oldPos;//start position for new target
int curPos;// the current position
int DTG;// distance to go to target
bool calibrating;// true = currently in calibrstion moder
bool CCW = 0; // rotating counder clockwise
int zeroCross;
bool debugOnceDisplay;

Encoder myEnc(ROT_ENCA_PIN, ROT_ENCB_PIN); // create an encoder called myEnc from the Encoder class specifing the connected pins (2&3)
AccelStepper stepper(AccelStepper::FULL4WIRE, 9, 7, 8, 6);  // create a stepper from AccelStepper class type full 4 wire  note pin connections are not in sequence. 

//next 2 lines FOR JMRI 
Auto485 bus(DE_PIN);                                        // Arduino pin 2 -> MAX485 DE and RE pins 
CMRI cmri(CMRI_ADDR, 24, 48, bus);                          // defaults to a SMINI at address CMRI_ADDR 24 in 48 out


//==================================MEMORY WRITE ========================
void memoryWrite(byte addr, int data) {
  EEPROM.write(addr, data >> 8);
  EEPROM.write(addr + 100, (data & 0xFF));
  delay(20);
}

// ===============================MEMORY READ=============================
int memoryRead(byte addr) {
  int rdata = 0xFF;
  rdata = EEPROM.read(addr);
  rdata = (rdata << 8) + (EEPROM.read(addr + 100) & 0xFF);
  return rdata;
}

//====================== CALIBRATE =================================================

void calibrate() {
  if (digitalRead(but[0]) == 0) { //but[0] = encoder push initiated 
    digitalWrite(LED, 0);
    delay(200);
    buttonTimeout = millis() + buttonDelay;
    calibrating = 0;
    while (digitalRead(but[0]) == 0) {
      if (millis() > buttonTimeout) {
        calibrating = 1; // long press detected

        if (DEBUG) Serial.println(" Set Up calibrating... ");
        digitalWrite(LED, 1); 
        findzero(-1, 3); //(move ccw 3 steps at a time)
        digitalWrite(LED, 1);
        myEnc.write(pos[oldButton]); // but[0]  (encoder) is the newButton the old button is the current turntable position 
        curPos = pos[oldButton];
        oldPos = curPos;
        oldButton = newButton;
        while (digitalRead(but[0]) == 0) {}  //wait for release if not already 
        delay(200);                          //debounce
        if (DEBUG) {
          Serial.print("Calibrating. moving to position ");
          Serial.println(myEnc.read());
        }
        //stepper.moveTo(pos[oldButton]);
        stepper.setSpeed(100);
        stepper.runToNewPosition(pos[oldButton]);
        // calibration setup now complete
      }
    }
  }
  while (calibrating) {
    digitalWrite(LED, 1);
    int newPos = myEnc.read();  //get current encoder pos
    if (DEBUG) {
      if (newPos != curPos) Serial.println(newPos);
    }
    stepper.moveTo(newPos);
    stepper.setSpeed(100);
    stepper.runSpeedToPosition();
    curPos = newPos;

    if (newButton > 0) oldButton = newButton;

    newButton = 0;
    buttonScan();
    if (newButton > 0) { //if not the encoder press
      oldButton = newButton;
      pos[newButton] = curPos;
      if (DEBUG) {
        Serial.print(" current position ");
        Serial.print(pos[newButton]);
        Serial.print(" to array index ");
        Serial.print(newButton);
      }
    }
    if (digitalRead(but[0]) == 0) {
      buttonTimeout = millis() + buttonDelay;
      while (digitalRead(but[0]) == 0) {
        if (millis() > buttonTimeout) {
          digitalWrite(LED, 0);
          delay(1000);
          digitalWrite(LED, 1);
          for (int i = 1; i <= 10; i++) {

            if (memoryRead(i) != pos[i]) {
              if (DEBUG) {
                Serial.print(" Saving... ");
                Serial.print(pos[i]);
                Serial.print(" to memory ");
                Serial.println(i);
              }
              memoryWrite(i, pos[i]);
              delay(20);
            }
          }
          while (digitalRead(but[0]) == 0) {}  // wait for release
        }
      }
      //      if (DEBUG) Serial.println(" Ending calibration. zeroing... ");
      //      findzero();
      //      if (oldButton == 0) oldButton = 1;
      //      newButton = oldButton;
      //      if (DEBUG) Serial.print(" newButton =  ");
      //      if (DEBUG) Serial.println(newButton);

      newButton = oldButton;

      calibrating = 0;
      stepper.setMaxSpeed(1000);
      digitalWrite(LED, 0);
      if (DEBUG) Serial.println(" Done cal... ");

      delay(20);  //debounce
    }
  }
}

//====================== FIND ZERO ANY WAY =================================================
void findzero(int direct, int rotSpeed) {  // use zerocross -1 for ccw and 1 for cw
  rotSpeed *= 100;
  digitalWrite(LED, 1);
  if (DEBUG) Serial.print("moving home to zero.  micro is at ");
  if (DEBUG) Serial.println(digitalRead(MICROSW));

  stepper.setMaxSpeed(rotSpeed);
  stepper.setSpeed(rotSpeed);
  long homing = 0;
  while (digitalRead(MICROSW) == 1) {
    stepper.move(homing);
    //printf("homing = %d\n",homing);
    //    if (DEBUG) {
    //      Serial.print("speed ");
    //      Serial.println (stepper.speed());
    //    }
    stepper.runSpeed();
    homing += direct;
    delay(3);
  }

  delay(1000);
  homing = 0;

  stepper.setMaxSpeed(100);
  stepper.setSpeed(100);
  while (digitalRead(MICROSW) == 0) {
    stepper.move(homing);
    stepper.run();
    delay(5);  // slow right down
    homing++;
  }
  if (DEBUG) Serial.println(" Found 0");
  curPos = 0;
  newPos = 0;
  myEnc.write(0);
  stepper.setCurrentPosition(0);
  digitalWrite(LED, 0);


  if (DEBUG) Serial.println("done find zero");
}


//====================== BUTTONSCAN =================================================
void buttonScan() {

  for (int i = 1; i <= 5; i++) {
    if (digitalRead(but[i]) == 0) {
      newButton = i;
      bool longPress = 0;
      delay(5);
      
      buttonTimeout = millis() + buttonDelay;  // start timer for long push
      while (digitalRead(but[i]) == 0) {       //the i'th button is pressed
        if (millis() > buttonTimeout) {
          digitalWrite(LED, HIGH);
          longPress = 1;
        }
      }
      //delay(50);
      digitalWrite(LED, 0);
      if (longPress && newButton > 0) {
        newButton = i + 5;
      }
      if (DEBUG) {
        printf("Button pressed. \nMoving from position %d to new position %d\n", oldButton, newButton);
      }
      //stepper.run();
      delay(5);  //debounce
      //stepper.run();
      break;
    }
  }
}
//====================== rotate =================================================
void rotate() {

  curPos = stepper.currentPosition();

  //printf("stepper position is %d\n",curPos);
  // complete any current move
  if (newButton != oldButton) {  //set a new target
    if (stepper.distanceToGo() != 0) {
      stepper.setAcceleration(400);
      stepper.stop();
      printf("interrupting move!\n ");
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      //curPos = stepper.currentPosition();
    }
    newPos = pos[newButton];
    curPos = curPos % ROTATION_STEPS;
    stepper.setCurrentPosition(curPos);


    if (DEBUG) {
      printf("\n\nCurrent  position %d, Moving to %d\n", curPos, newPos);
      printf("stepper position is %d\n", stepper.currentPosition());
    }

    oldButton = newButton;
    zeroCross = 0;
    DTG = newPos - curPos;
    if (DEBUG) {
      debugOnceDisplay = 1;
      printf("1] simple rotate DTG = %d\n", DTG);
    }
    // following section determines optimal driection fot the current move.
    if (DTG < (0 - HALF_ROTATION_STEPS)) {
      //rotating anticw more than 180 degrees
      //if (DTG + ROTATION_STEPS < abs(DTG)) {
      DTG += ROTATION_STEPS;
      if (DTG + curPos > ROTATION_STEPS) {
        zeroCross = 1;  //has crossed zero CW
      }
      if (DEBUG) {
        printf("2] Shorter route X zero CW DTG = %d\n", DTG);
      }
    }
    if (DTG > HALF_ROTATION_STEPS) {
      DTG = -(ROTATION_STEPS - DTG);
      if (curPos < abs(DTG)) {
        zeroCross = -1;  //has crossed zero CCW
      }
      if (DEBUG) {
        printf("3] Shorter route X zero CCW DTG = %d\n", DTG);
      }
    }
    CCW = 0;
    //take up gearbox slack
    if (DTG < 0) {
      DTG -= CCW_OVERSHOOT;
      CCW = 1;
      if (DEBUG) {
        printf("4] Take up gearbox slack DTG = %d\n", DTG);
      }
    }
    if (DEBUG) {
      Serial.print("zeroCross= ");
      Serial.println(zeroCross);
    }

    if (zeroCross == 0) {  // possibly convert to case select
    }
    if (zeroCross == -1) {
      findzero(-1, 2);
      DTG = 0 - (ROTATION_STEPS - pos[newButton]);
      DTG -= CCW_OVERSHOOT;
      CCW = 1;
    }
    if (zeroCross == 1) {
      findzero(1, 2);
      DTG = pos[newButton];
    }
    stepper.setSpeed(600);
    stepper.setAcceleration(10);
    stepper.move(DTG);
    if (DEBUG) {
      //Serial.println("");
      Serial.print("CurPos ");
      Serial.print(curPos);
      Serial.print(" newPos ");
      Serial.print(newPos);
      Serial.print(" Move DTG ");
      Serial.print(DTG);
      if (CCW) {
        Serial.print(" Anti ");
      }
      Serial.print(" Clockwise ");
      Serial.println("");
    }
  }

  digitalWrite(LED, 0);
  if (stepper.distanceToGo() != 0) {
    stepper.run();
    digitalWrite(LED, 1);

  } else {
    ///printf("cleaning up...no DTG [ln363] CCW = %d, curpos = %d,\n", CCW, curPos);
    // tidy up
    if (CCW == 1) {
      /*its this next bit that may need looking at
        following a ccw move there is a chance the tt pos is negative
        need to convert neg to positive*/
      if (curPos < 0) {
        printf("converting negative curPos %d to ", curPos);
        curPos = ROTATION_STEPS + curPos;
        stepper.setCurrentPosition(curPos);
        printf("%d \n", curPos);
      }
      int tnewPos = curPos + CCW_OVERSHOOT;
      DTG = CCW_OVERSHOOT;
      CCW = 0;
      stepper.setSpeed(600);
      stepper.setAcceleration(10);
      stepper.move(DTG);
      curPos = tnewPos;

      if (DEBUG) {
        Serial.println("line 382 cleaning up CCW");
        Serial.print(DTG);
        Serial.print(" Returning Overshoot ");
        Serial.println("");
      }
    }
    if (DEBUG) {
      if (debugOnceDisplay) {
        printf("Done. curPos = %d, ZeroX = %d, New curPos = %d. \n", curPos, zeroCross, pos[newButton]);
        //curPos = pos[newButton];
        //stepper.setCurrentPosition(curPos);

        printf("\n\n");
        debugOnceDisplay = 0;
      }
    }
  }
}

//======================= getData ====================================

//next function(16 lines) FOR JMRI 
void getData() {
  for (int i = 0; i < 10; i++) {
    cmri.set_bit(i, 0);
  }
  // if ( newButton != oldButton) {
  cmri.set_bit(newButton - 1, 1);  //set CMRI sensor for newButton bit
    //cmri.set_bit(oldButton-1,0); // cancel old button bit
  // }
  cmri.process();  // get JMRI data via CMRI bits

  for (int i = 0; i < 10; i++) {
    if (cmri.get_bit(i) == 1) {
      newButton = i + 1;
      break;
    }
  }
}

//=======================================================================
//====================== SETUP =================================================
//=======================================================================

void setup() {

 
  Serial.begin(19200);
 
  if (DEBUG) Serial.println("\nTurntableNB12 non blocking Steve Lomax 28/05/24\nadding RS485\n");
  if (DEBUG) Serial.println("CMRI address 1\n");
  pinMode(but[0], INPUT_PULLUP);  //encoder push
  pinMode(but[1], INPUT_PULLUP);
  pinMode(but[2], INPUT_PULLUP);
  pinMode(but[3], INPUT_PULLUP);
  pinMode(but[4], INPUT_PULLUP);
  pinMode(but[5], INPUT_PULLUP);
  pinMode(MICROSW, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  for (int i = 1; i <= 10; i++) {
    pos[i] = memoryRead(i);  // load in button position values
    if (pos[i] < 0) {        //invalid value
      pos[i] = 0;
      memoryWrite(i, 0);
    }
    printf(" reading button %d, value %d.\n", i, pos[i]);
  }
  Serial.end();
  stepper.setMaxSpeed(1000);
  findzero(-1, 3);
  oldButton = 10;
  newButton = 1;

  myEnc.write(0);
  stepper.setCurrentPosition(0);
  curPos = 0;
  bus.begin(19200);
  delay(200);
}
//=======================================================================
//====================== LOOP =================================================
//=======================================================================

void loop() {

  if (!calibrating) {
    rotate();

    //next line FOR JMRI 
    if( stepper.distanceToGo() == 0) getData();
  }

  buttonScan();
  calibrate();
}
