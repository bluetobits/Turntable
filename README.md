# Turntable

## TurntableBeta, based on TurntableNB12

## ©Steve Lomax 22/11/2024 not for commercial purposes.

 this is, and will always remain, a work in progress. This is a working version,

### Features:

*   10 preset positions on 5 buttons (long/short press).  
*   this will permit either end of the turntable bridge to face each of 5 track rays  
* movement may be controlled by manual push switch and/or from JMRI over C/MRI. JMRI  or C/MRI  is not essential.  
*   Manual rotation and position setting (calibrating) is controlled by the encoder  
*   To offset any backlash and assist in alignment the final approach to each position will always be clockwise.  
*   (an anti-clockwise move will overshoot a few degrees  and creep back clockwise)  
*   Turntable  movements have acceleration and deceleration curves settable prior to upload  
*   The Turntable will take shortest route to the target position from the current position  
*   The target position may be changed mid rotation from the panel switches JMRI control is only possible when the turntable is stationary.  
*   the turntable will re-set the zero position each time it passes the zero position and each time the calibration (encoder long push) button is pressed  
*   When in calibration mode, pressing any button (short or long press) will record the current position into that button memory overwriting the previous setting.

  all switches and encoder connect pin to ground  
  all LEDs connect pin to ground via limiting resistor (typ 200R)  
  microswitch fitted to turntable to mark 'position 0' which may be anywhere,  
  but ideally somewhere between the last and first track rays.

##   Hardware:

*   Turntable  
*   Stepper motor 28BYJ-48 and ULN2003 driver module  
*   Micro switch positioned to trigger when the turntable is at 'home' position (zero) Ideally this would be anticlockwise before position 1  
*   The software will allow for switch dwell angle so cam operation is possible.  
*   6 push buttons (5 rays and reset)  
*   rotary Encoder module (puluap resistors and debounce capacitor fitted)  
*   LED with series resistor for 5V  
*   5V 1 amp power supply  
*   Arduino  
*   Connecting leads  
*   Mounting panel

##   CONNECTIONS

  pin 0 DI on 485 Module  
  pin 1 RO on 485 Module  
  pin 2 encoder data 1  
  pin 3 encoder data 2  
  pin 4 DE/RE (joined)  on 485 Module  
  Pin 5 encoder push button  
    Pins 6,7,8,9 to stepper 1,2,3,4  
  pin 10 limit (position 0\) microswitch  
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
