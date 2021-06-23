#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>

class WiFi_Handler {
  
  private:
    char dataArray[14];
    
  public:
    WiFi_Handler();
    void connectToWiFi();
    void printWiFiStatus();
    void beginUDP();
    bool sendUDP(char dataArray[]);
    void init();
};

#endif
