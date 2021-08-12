#include "Input_Handler.h"

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];            // temporary array for use when parsing

char stepDirection[numChars] = {0};
boolean newData = false;

static boolean recvInProgress = false;
static byte ndx = 0;
char startMarker = '<';
char endMarker = '>';
char rc;

Input_Handler::Input_Handler() {
  init();
}
void Input_Handler::init() {
}

//======== For detecting start/end markers in Serial input ========//
void Input_Handler::recvWithStartEndMarkers() {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    } else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//======== For parsing Serial input data ========//
void Input_Handler::parseData() {      // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");
  strcpy(stepDirection, strtokIndx);
  strtokIndx = strtok(NULL, ",");
  steps = atoi(strtokIndx);
}

//============= To show parsed Serial inputs ==============//
void Input_Handler::showParsedData() {
  Serial.print("Direction ");
  Serial.println(stepDirection);
  Serial.print("Steps ");
  Serial.println(steps);
}

void Input_Handler::displayInstructions() {
  // For Serial Monitor
  Serial.println("Enter commands in the form of <stepDirection,steps>");
  Serial.println("valid stepDirections are {up,down,left,right}");
  Serial.println();
}
