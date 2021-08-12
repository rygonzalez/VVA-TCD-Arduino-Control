#ifndef INPUT_HANDLER_H
#define INPUT_HANDLER_H

#include <Arduino.h>

class Input_Handler {
  private:
    void init();
    void parseData();
    void showParsedData();
  public:
    int steps;
    Input_Handler();
    void recvWithStartEndMarkers();
    void displayInstructions();
};

#endif
