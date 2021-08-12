#include "WiFi_Handler.h"
#include "Stepper_Motor.h"
#include "Input_Handler.h"

// analog reads instantiations
#define TCD_PIN A0                 //likely TCD (+/-5V)
//int analogPin1 = A1;               //likely chair velocity (+/-2.5V)
//int analogPin2 = A2;               //likely chair position (+/-5V)

// UDP Instantiations
WiFi_Handler wifiHandler;
bool flagUDP = false;
String dataString;
char* dataArray;

// limit switch instantiations
#define UP_PIN 3
#define DOWN_PIN 4
#define LEFT_PIN 5
#define RIGHT_PIN 6
bool upTrigger = 0;
bool downTrigger = 0;
bool leftTrigger = 0;
bool rightTrigger = 0;

// timer instantiations
bool actuateMotors = false;

// motor instantiations
/*
   Motor cable order
   A_bar / A / B_bar / B
   Black / Red / Yellow / Green
*/
// for our up/down motor
#define A1_bar    7                  // the pin connected to the wire A- of the coil A (or to the H-bridge pin controlling the same wire)
#define A1        8                  // the pin connected to the wire A of the coil A (or to the H-bridge pin controlling the same wire)
#define B1_bar    9                  // the pin connected to the wire B- of the coil B (or to the H-bridge pin controlling the same wire)
#define B1        10                 // the pin connected to the wire B of the coil B (or to the H-bridge pin controlling the same wire)
// for our left/right motor
#define A2_bar    11                 // the pin connected to the wire A- of the coil A (or to the H-bridge pin controlling the same wire)
#define A2        12                 // the pin connected to the wire A of the coil A (or to the H-bridge pin controlling the same wire)
#define B2_bar    13                 // the pin connected to the wire B- of the coil B (or to the H-bridge pin controlling the same wire)
#define B2        14                 // the pin connected to the wire B of the coil B (or to the H-bridge pin controlling the same wire)
// motor control variables
#define step_delay        1000                // smaller values may make the motor produce more speed and less torque
// Note: approximate distance is 6.2um/step for current motors
Stepper_Motor x_motor(A1, A1_bar, B1, B1_bar, step_delay, upTrigger, downTrigger);
Stepper_Motor y_motor(A2, A2_bar, B2, B2_bar, step_delay, leftTrigger, rightTrigger);

// serial parsing instantiations
Input_Handler inputHandler;

void TC_Setup() {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |                 // Enable GCLK0 for TC4 and TC5
                      GCLK_CLKCTRL_GEN_GCLK0 |             // Select GCLK0 at 48MHz
                      GCLK_CLKCTRL_ID_TC4_TC5;             // Feed GCLK0 output to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);                       // Wait for synchronization

  TC4->COUNT16.CC[0].reg = 47999;                          // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_OVF;             // Enable TC4 overflow (OVF) interrupts

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC |     // Reset timer on the next prescaler clock
                            TC_CTRLA_PRESCALER_DIV1 |      // Set prescaler to 8, 48MHz/8 = 6MHz
                            TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode
                            TC_CTRLA_MODE_COUNT16;         // Set the timer to 16-bit mode

  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC5->COUNT16.CC[0].reg = 47999;                          // Set the TC5 CC0 register as the TOP value in match frequency mode
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  NVIC_SetPriority(TC5_IRQn, 1);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC5_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  TC5->COUNT16.INTENSET.reg = TC_INTENSET_OVF;             // Enable TC5 overflow (OVF) interrupts

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC |     // Reset timer on the next prescaler clock
                            TC_CTRLA_PRESCALER_DIV1 |      // Set prescaler to 8, 48MHz/8 = 6MHz
                            TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC5 into match frequency (MFRQ) mode
                            TC_CTRLA_MODE_COUNT16;         // Set the timer to 16-bit mode

  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                       // Enable the TC4 timer
  TC5->COUNT16.CTRLA.bit.ENABLE = 1;                       // Enable the TC5 timer

  while (TC4->COUNT16.STATUS.bit.SYNCBUSY && TC5->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization
}

// TC4 timing instantiations
int its = 0; // for storing values before averaging
int values[1000];
int currentValue;
int averages[2];
int sum = 0;
int avg = 0;

//PI constants
double kp = 2;
double ki = 5;

//PI instantiations
double error, cumError;
double output;
boolean overShoot = false;
boolean XYdirection = true; //using a boolean to alternate between X/Y direction (T/F)

double setPoint = 1023;
double prevError = 1023;

void TC4_Handler() { // Interrupt Service Routine (ISR) for timer TC4, handles collecting information
  if (!actuateMotors) {
    currentValue = analogRead(TCD_PIN); // reads TCD pin value every 1ms

    error = setPoint - currentValue; // calculate "error" with value
    cumError += error;
    output = kp*error + ki*cumError;

    values[its] = currentValue; // store value in array
    if (its >= 1000) { // every 1s take average of values, perform PI tasks
      // value averaging & UDP
      for (int m = 0; m < 500; m++) {
        sum = sum + values[m];
      }
      avg = sum / 500;
      averages[0] = avg;
      avg = 0;
      sum = 0;
      for (int n = 500; n < 1000; n++) {
        sum = sum + values[n];
      }
      avg = sum / 500;
      averages[1] = avg;
      avg = 0;
      sum = 0;
      flagUDP = true;
      its = 0; // reset values to begin new "cycle"

      // controller tasks
      if(output > prevError && !overShoot) { // make sure we dont overshoot and spiral out of control
        overShoot = true;
      } else if(!overShoot) {
        XYdirection = !XYdirection; //flip XY boolean to alternate motor actuation axis
        actuateMotors = true; // signal we're going to actuate motors
        prevError = output;
      }
    }
  }
  TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;             // Clear the OVF interrupt flag
}

void TC5_Handler() { // Interrupt Service Routine (ISR) for timer TC5, handles PI Control
  
  TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;  // Clear the OVF interrupt flag
}

void limitUp() {
  if (digitalRead(UP_PIN) == HIGH) {
    upTrigger = 0;
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    upTrigger = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void limitDown() {
  if (digitalRead(DOWN_PIN) == HIGH) {
    downTrigger = 0;
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    downTrigger = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void limitLeft() {
  if (digitalRead(LEFT_PIN) == HIGH) {
    leftTrigger = 0;
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    leftTrigger = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void limitRight() {
  if (digitalRead(RIGHT_PIN) == HIGH) {
    rightTrigger = 0;
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    rightTrigger = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void setup() {
  Serial.begin(9600);

  // WiFi initialization
  wifiHandler.connectToWiFi();
  wifiHandler.printWiFiStatus();
  wifiHandler.beginUDP();

  // For limit switch interrupts
  digitalWrite(UP_PIN, HIGH);
  digitalWrite(DOWN_PIN, HIGH);

  // These require different syntax for arduino uno rev2 wifi, differ per board. Check:
  // https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  attachInterrupt(digitalPinToInterrupt(UP_PIN), limitUp, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DOWN_PIN), limitDown, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN), limitLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_PIN), limitRight, CHANGE);

  // Built-in LED to flash when we hit any of our limit switches
  pinMode(LED_BUILTIN, OUTPUT);

  // sets timer to call sendUdp function to send data every 1 millis or 1kHz
  // TC_Setup();

  inputHandler.displayInstructions();
}

double controllerScaler = 0.1; //scales output of controller to palatable step sizes

void loop() {
  // Scrape for data from Serial Monitor
  inputHandler.recvWithStartEndMarkers();

  // send UDP info (need to figure out where to put analog info parsing)
  if (flagUDP) {
    dataString = String(averages[0]) + String(averages[1]);
    dataString.toCharArray(dataArray, 14);
    wifiHandler.sendUDP(dataArray);
    flagUDP = false;
  }

  if(actuateMotors){
    switch (XYdirection) {
      case 0: // move in Y direction
        y_motor.stepForward(int(output*controllerScaler));
        actuateMotors = false;
        break;
      case 1: // move in X direction
        x_motor.stepForward(int(output*controllerScaler));
        actuateMotors = false;
        break;
    }
  }
}
