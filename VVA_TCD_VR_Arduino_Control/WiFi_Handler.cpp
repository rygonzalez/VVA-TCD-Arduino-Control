#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "WiFi_Handler.h"
#include "arduino_network_config.h"


///////please enter your network data in the arduino_network_config.h file
char ssid[] = LOCAL_SSID;            // your network SSID (name)
char pass[] = LOCAL_PASS;            // your network password (use for WPA, or use as key for WEP)
unsigned int localPort = LOCAL_PORT;  // assign local port to Arduino
char hostIP[] = HOST_IP;              // receiving host's IP address
unsigned int port = HOST_PORT;        // receiving host's port
int status = WL_IDLE_STATUS;          // the WiFi radio's status
WiFiUDP Udp;                          //instantiate UDP object

WiFi_Handler::WiFi_Handler() {

  init();
}
void WiFi_Handler::init() {
}

//connects Arduino to WiFi in start()
void WiFi_Handler::connectToWiFi() {
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("Connected to wifi");
}

//Network feedback data
void WiFi_Handler::printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void WiFi_Handler::beginUDP() {
  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}

//sends packet of data
bool WiFi_Handler::sendUDP(char dataArray[]) {
  Udp.beginPacket(hostIP, port);
  Udp.write(dataArray);
  Udp.endPacket();
  Serial.print("data sent: ");
  Serial.println(dataArray);
  return true; // to repeat the action - false to stop
}
