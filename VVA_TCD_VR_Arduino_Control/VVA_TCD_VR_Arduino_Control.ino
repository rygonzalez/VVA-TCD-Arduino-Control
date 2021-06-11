#include <WiFiNINA.h>
#include <WiFiUdp.h>

#include "arduino_network_config.h"

///////please enter your network data in the arduino_netowrk_config.h file
char ssid[] = LOCAL_SSID;            // your network SSID (name)
char pass[] = LOCAL_PASS;            // your network password (use for WPA, or use as key for WEP)

unsigned int localPort = LOCAL_PORT;  // assign local port to Arduino

char hostIP[] = HOST_IP;              // receiving host's IP address
unsigned int port = HOST_PORT;        // receiving host's port

int status = WL_IDLE_STATUS;          // the WiFi radio's status
WiFiUDP Udp;                          //instantiate UDP object

String dataString;
char dataArray[14];

int analogPin0 = A0;                  //likely TCD (+/-5V)
int analogPin1 = A1;                  //likely chair velocity (+/-2.5V)
int analogPin2 = A2;                  //likely chair position (+/-5V)

int input0 = 0;                       //variables to store read values (0-5V)(0-1023)
int input1 = 0;
int input2 = 0;

void setup() {
  // Initialize serial and wait for port 115200 to open:
  Serial.begin(115200);

  // sets timer to call sendUdp function to send data every 1 millis or 1kHz
  cli();
  TCB0.CCMP = 16000; // Value to compare/capture with. This is 1/1000th total counts, so 1kHz
  TCB0.CTRLA = (1 << TCB_CLKSEL_CLKDIV1_gc); // enable clock
  TCB0.CTRLA = (1 << TCB_ENABLE_bm); // enable timer
  // TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
  TCB0.CTRLB = (1 << TCB_CNTMODE_INT_gc); // Use timer compare mode 
  // TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.INTCTRL = (1 << TCB_CAPT_bm); // Enable the interrupt 
  // TCB0.INTCTRL = TCB_CAPT_bm; 
  sei();

  // connects arduino to local WiFi
  connectToWiFi(); //COMMENT IF NOT USING WIFI
}

ISR(TCB0_INT_vec) {
  sendUDP(dataArray);
}

void loop() {
  Serial.println(TCB0.CNT); // prints timer value from CNT register
}

//sends packet of data
bool sendUdp(char dataArray[]) {

  // Read pin values (0-1023)
  input0 = analogRead(analogPin0);
  input1 = analogRead(analogPin1);
  input2 = analogRead(analogPin2);

  //dataString = String(input0) + "$" + String(input1) + "$" + String(input2); // when transitioning to three analog pins, uncomment + remove line below
  dataString = String(input0);
  dataString.toCharArray(dataArray, 14);
  Udp.beginPacket(hostIP, port);
  Udp.write(dataArray);
  Udp.endPacket();

  return true; // to repeat the action - false to stop
}

//connects Arduino to WiFi in start()
void connectToWiFi() {
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

    //    // wait 5 seconds for connection:
    //    delay(5000);
  }
  Serial.println("Connected to wifi");
  printWifiStatus();

  Serial.println("\nStarting connection to server...");
  // if you get a connection, report back via serial:
  Udp.begin(localPort);
}


//Network feedback data
void printWifiStatus() {
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
