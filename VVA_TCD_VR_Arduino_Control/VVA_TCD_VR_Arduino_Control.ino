#include "WiFi_Handler.h"
#include "Limit_Switch.h"

String dataString;
char dataArray[14];
bool flagUDP = false;

int analogPin0 = A0;                  //likely TCD (+/-5V)
int analogPin1 = A1;                  //likely chair velocity (+/-2.5V)
int analogPin2 = A2;                  //likely chair position (+/-5V)

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

/*
   Serial Parsing instantiations
   includes variables for storing data, holding parsed data, and new data input checks
*/

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];            // temporary array for use when parsing

char stepDirection[numChars] = {0};
int steps = 0;

boolean newData = false;

byte up = 18;
bool upTrigger = 0;

int x = 0;
int y = 0;

WiFi_Handler handler;

void setup() {
  Serial.begin(115200);

  // WiFi initialization
  handler.connectToWiFi();
  handler.printWiFiStatus();
  handler.beginUDP();

  // Testing sendUDP command of handler
  dataString = "testdata!!!!!!";
  dataString.toCharArray(dataArray, 14);
  handler.sendUDP(dataArray);

//  // setting our motor pin states as internal pull-up until ready to actuate
//  MotorsOff1();
//  MotorsOff2();
//
//  // For Serial Monitor
//  Serial.println("Enter commands in the form of <stepDirection,steps>");
//  Serial.println("valid stepDirections are {up,down,left,right}");
//  Serial.println();
//
  // For limit switch interrupts
  digitalWrite(up,    HIGH);
//
//  // These require different syntax for arduino uno rev2 wifi, differ per board. Check:
//  // https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  attachInterrupt(digitalPinToInterrupt(up), limitUp,    CHANGE);
//
//  // Built-in LED to flash when we hit any of our limit switches
//  pinMode(LED_BUILTIN, OUTPUT);

  // sets timer to call sendUdp function to send data every 1 millis or 1kHz
  //TC_Setup();
}



//int x = 0; // for timing
//int y = 0;
//int i = 0; // for storing values before averaging
//int values[500];
//int sum = 0;
//int avg = 0;
//String d1 = "";
//String d2 = "";
//String d3 = "";
//String d4 = "";

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

  TC5->COUNT16.CC[0].reg = 23999;                          // Set the TC5 CC0 register as the TOP value in match frequency mode
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  NVIC_SetPriority(TC5_IRQn, 1);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC5_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  TC5->COUNT16.INTENSET.reg = TC_INTENSET_OVF;             // Enable TC5 overflow (OVF) interrupts

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC |     // Reset timer on the next prescaler clock
                            TC_CTRLA_PRESCALER_DIV1 |      // Set prescaler to 8, 48MHz/8 = 6MHz
                            TC_CTRLA_WAVEGEN_MFRQ |        // Put the timer TC4 into match frequency (MFRQ) mode
                            TC_CTRLA_MODE_COUNT16;         // Set the timer to 16-bit mode

  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                       // Enable the TC4 timer
  TC5->COUNT16.CTRLA.bit.ENABLE = 1;                       // Enable the TC5 timer

  while (TC4->COUNT16.STATUS.bit.SYNCBUSY && TC5->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization
}

void TC4_Handler()                                       // Interrupt Service Routine (ISR) for timer TC4
{
//  values[i] = analogRead(A0);
//  if (i >= 1000){ // every 1s take average of values
//    for(int n = 0; n < 499; n++) {
//      sum = sum + values[n];
//    }
//    avg = sum/1000;
//    sum = 0;
//    i = 0;
//  }
//  x=x+1;
//  if(x % 1000 == 0 && x > 0){
//    Serial.println(avg);
//  }

  x++;
  if (x % 1000 == 0) {
    Serial.print("x = ");
    Serial.println(x);
    Serial.print("y = ");
    Serial.println(y);
  }
  TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;             // Clear the OVF interrupt flag
}

void TC5_Handler()                                       // Interrupt Service Routine (ISR) for timer TC5
{
//  d1 = values[i-1];
//  d2 = values[i];
//  dataString = d1 + "$" + d2 + "$" + d3;
//  dataString.toCharArray(dataArray, 19);
//  sendUDP(dataArray);
//  y=y+1;
//  if(y % 333 == 0 && x > 0){
//    Serial.println(dataArray);
//  }

//  if (y % 1000 == 0) {
//    Serial.print("x = ");
//    Serial.println(x);
//  }

  y++;
  TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;             // Clear the OVF interrupt flag
}

void loop() {
//  // Scrape for data from Serial Monitor
//  recvWithStartEndMarkers();
  // If new input, begin subroutine
//  if (newData == true) {
//    strcpy(tempChars, receivedChars);
//    // this temporary copy is necessary to protect the original data
//    //   because strtok() used in parseData() replaces the commas with \0
//    parseData();
//    showParsedData();
//    newData = false; // ensures we don't have infinite loop
//    // Comparing input to determine direction of operation (if any) and instigating operation
//    if (strcmp(stepDirection, "up") == 0) {
//      StepUp();
//    }
//    if (strcmp(stepDirection, "down") == 0) {
//      StepDown();
//    }
//    if (strcmp(stepDirection, "left") == 0) {
//      StepLeft();
//    }
//    if (strcmp(stepDirection, "right") == 0) {
//      StepRight();
//    }
//  }
}

//======== For limit switch interrupts ==========//
void limitUp() {
  if (digitalRead(up) == HIGH) {
    upTrigger = 0;
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    upTrigger = 1;
    digitalWrite(LED_BUILTIN, HIGH);
  }
}


////========== For detecting start/end markers in Serial input ===========//
//void recvWithStartEndMarkers() {
//  static boolean recvInProgress = false;
//  static byte ndx = 0;
//  char startMarker = '<';
//  char endMarker = '>';
//  char rc;
//
//  while (Serial.available() > 0 && newData == false) {
//    rc = Serial.read();
//    if (recvInProgress == true) {
//      if (rc != endMarker) {
//        receivedChars[ndx] = rc;
//        ndx++;
//        if (ndx >= numChars) {
//          ndx = numChars - 1;
//        }
//      } else {
//        receivedChars[ndx] = '\0'; // terminate the string
//        recvInProgress = false;
//        ndx = 0;
//        newData = true;
//      }
//    } else if (rc == startMarker) {
//      recvInProgress = true;
//    }
//  }
//}

////=========== For parsing Serial input data ============//
//void parseData() {      // split the data into its parts
//  char * strtokIndx; // this is used by strtok() as an index
//  strtokIndx = strtok(tempChars, ",");
//  strcpy(stepDirection, strtokIndx);
//  strtokIndx = strtok(NULL, ",");
//  steps = atoi(strtokIndx);
//}

////============= To show parsed Serial inputs ==============//
//void showParsedData() {
//  Serial.print("Direction ");
//  Serial.println(stepDirection);
//  Serial.print("Steps ");
//  Serial.println(steps);
//}


////============ for motor control ================//
//void MotorsOn1() { // turns on motors, to start operating
//  pinMode(A1, OUTPUT);
//  pinMode(A1_bar, OUTPUT);
//  pinMode(B1, OUTPUT);
//  pinMode(B1_bar, OUTPUT);
//}
//
//void MotorsOff1() { // turns off motors, to stop current draw. Useful after moving, or when not operating
//  pinMode(A1, INPUT);
//  digitalWrite(A1, HIGH);
//  pinMode(A1_bar, INPUT);
//  digitalWrite(A1_bar, HIGH);
//  pinMode(B1, INPUT);
//  digitalWrite(B1, HIGH);
//  pinMode(B1_bar, INPUT);
//  digitalWrite(B1_bar, HIGH);
//}
//
//void MotorsOn2() { // turns on motors, to start operating
//  pinMode(A2, OUTPUT);
//  pinMode(A2_bar, OUTPUT);
//  pinMode(B2, OUTPUT);
//  pinMode(B2_bar, OUTPUT);
//}
//
//void MotorsOff2() { // turns off motors, to stop current draw. Useful after moving, or when not operating
//  pinMode(A2, INPUT);
//  digitalWrite(A2, HIGH);
//  pinMode(A2_bar, INPUT);
//  digitalWrite(A2_bar, HIGH);
//  pinMode(B2, INPUT);
//  digitalWrite(B2, HIGH);
//  pinMode(B2_bar, INPUT);
//  digitalWrite(B2_bar, HIGH);
//}
//
//void StepUp() {
//  MotorsOn1();
//  for (int i = 0; i < (steps / 4) ; i++) {
//    if (upTrigger) {
//      Serial.println("Upper limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, HIGH);
//    digitalWrite(A1_bar, LOW);
//    digitalWrite(B1, HIGH);
//    digitalWrite(B1_bar, LOW);
//    delayMicroseconds (step_delay);
//
//    if (upTrigger) {
//      Serial.println("Upper limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, LOW);
//    digitalWrite(A1_bar, HIGH);
//    digitalWrite(B1, HIGH);
//    digitalWrite(B1_bar, LOW);
//    delayMicroseconds (step_delay);
//
//    if (upTrigger) {
//      Serial.println("Upper limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, LOW);
//    digitalWrite(A1_bar, HIGH);
//    digitalWrite(B1, LOW);
//    digitalWrite(B1_bar, HIGH);
//    delayMicroseconds (step_delay);
//
//    if (upTrigger) {
//      Serial.println("Upper limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, HIGH);
//    digitalWrite(A1_bar, LOW);
//    digitalWrite(B1, LOW);
//    digitalWrite(B1_bar, HIGH);
//    delayMicroseconds (step_delay);
//  }
//  Serial.println("StepUp Subroutine Complete");
//  MotorsOff1();
//}
//
//void StepDown() {
//  MotorsOn1();
//  for (int i = 0; i < (steps / 4); i++) {
//    if (downTrigger) {
//      Serial.println("Lower limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, HIGH);
//    digitalWrite(A1_bar, LOW);
//    digitalWrite(B1, LOW);
//    digitalWrite(B1_bar, HIGH);
//    delayMicroseconds (step_delay);
//
//    if (downTrigger) {
//      Serial.println("Lower limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, LOW);
//    digitalWrite(A1_bar, HIGH);
//    digitalWrite(B1, LOW);
//    digitalWrite(B1_bar, HIGH);
//    delayMicroseconds (step_delay);
//
//    if (downTrigger) {
//      Serial.println("Lower limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, LOW);
//    digitalWrite(A1_bar, HIGH);
//    digitalWrite(B1, HIGH);
//    digitalWrite(B1_bar, LOW);
//    delayMicroseconds (step_delay);
//
//    if (downTrigger) {
//      Serial.println("Lower limit switch detects collision");
//      break;
//    }
//    digitalWrite(A1, HIGH);
//    digitalWrite(A1_bar, LOW);
//    digitalWrite(B1, HIGH);
//    digitalWrite(B1_bar, LOW);
//    delayMicroseconds (step_delay);
//  }
//  Serial.println("StepDown Subroutine Complete");
//  MotorsOff1();
//}
//
//void StepLeft() {
//  MotorsOn2();
//  for (int i = 0; i < (steps / 4) ; i++) {
//    if (leftTrigger) {
//      Serial.println("Left limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, HIGH);
//    digitalWrite(A2_bar, LOW);
//    digitalWrite(B2, HIGH);
//    digitalWrite(B2_bar, LOW);
//    delayMicroseconds (step_delay);
//
//    if (leftTrigger) {
//      Serial.println("Left limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, LOW);
//    digitalWrite(A2_bar, HIGH);
//    digitalWrite(B2, HIGH);
//    digitalWrite(B2_bar, LOW);
//    delayMicroseconds (step_delay);
//
//    if (leftTrigger) {
//      Serial.println("Left limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, LOW);
//    digitalWrite(A2_bar, HIGH);
//    digitalWrite(B2, LOW);
//    digitalWrite(B2_bar, HIGH);
//    delayMicroseconds (step_delay);
//
//    if (leftTrigger) {
//      Serial.println("Left limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, HIGH);
//    digitalWrite(A2_bar, LOW);
//    digitalWrite(B2, LOW);
//    digitalWrite(B2_bar, HIGH);
//    delayMicroseconds (step_delay);
//  }
//  Serial.println("StepLeft Subroutine Complete");
//  MotorsOff2();
//}
//
//void StepRight() {
//  MotorsOn2();
//  for (int i = 0; i < (steps / 4); i++) {
//    if (rightTrigger) {
//      Serial.println("Right limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, HIGH);
//    digitalWrite(A2_bar, LOW);
//    digitalWrite(B2, LOW);
//    digitalWrite(B2_bar, HIGH);
//    delayMicroseconds (step_delay);
//
//    if (rightTrigger) {
//      Serial.println("Right limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, LOW);
//    digitalWrite(A2_bar, HIGH);
//    digitalWrite(B2, LOW);
//    digitalWrite(B2_bar, HIGH);
//    delayMicroseconds (step_delay);
//
//    if (rightTrigger) {
//      Serial.println("Right limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, LOW);
//    digitalWrite(A2_bar, HIGH);
//    digitalWrite(B2, HIGH);
//    digitalWrite(B2_bar, LOW);
//    delayMicroseconds (step_delay);
//
//    if (rightTrigger) {
//      Serial.println("Right limit switch detects collision");
//      break;
//    }
//    digitalWrite(A2, HIGH);
//    digitalWrite(A2_bar, LOW);
//    digitalWrite(B2, HIGH);
//    digitalWrite(B2_bar, LOW);
//    delayMicroseconds (step_delay);
//  }
//  Serial.println("StepRight Subroutine Complete");
//  MotorsOff2();
//}
