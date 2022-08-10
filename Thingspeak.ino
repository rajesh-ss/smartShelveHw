/*
 WizFi360 example: Thingspeak

 This sketch Request to thingspeak using a WizFi360 module to
 perform a simple store and search data.
*/

#include <stdlib.h>
#include <DHT.h>
#define pingPin 7 // Trigger Pin of Ultrasonic Sensor
#define echoPin 6 // Echo Pin of Ultrasonic Sensor
#include "WizFi360.h"

// setup according to the device you use
#define WIZFI360_EVB_PICO

// Emulate Serial1 on pins 6/7 if not present
// remember that the wiznet is connected to 4 and 5 not 6 and 7
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
#if defined(ARDUINO_MEGA_2560)
SoftwareSerial Serial1(4, 5); // RX, TX
#elif defined(WIZFI360_EVB_PICO)
SoftwareSerial Serial2(4, 5); // RX, TX
#endif
#endif

/* Baudrate */
#define SERIAL_BAUDRATE   115200
#if defined(ARDUINO_MEGA_2560)
#define SERIAL1_BAUDRATE  115200
#elif defined(WIZFI360_EVB_PICO)
#define SERIAL2_BAUDRATE  115200
#endif

/* Sensor */
#define DHTTYPE DHT11
#define DHTPIN SDA
#define CDSPIN A0

/* Wi-Fi info */
char ssid[] = "oops";       // your network SSID (name)
char pass[] = "12345678";   // your network password

int status = WL_IDLE_STATUS;  // the Wifi radio's status

char server[] = "api.thingspeak.com"; // server address
String apiKey ="C3IE9V05E06LBR27";                    // apki key

// sensor buffer
char temp_buf[10];
char humi_buf[10];
char cds_buf[10];

unsigned long lastConnectionTime = 0;         // last time you connected to the server, in milliseconds
const unsigned long postingInterval = 30000L; // delay between updates, in milliseconds

// Initialize the Ethernet client object
WiFiClient client;
// Initialize the DHT object
DHT dht(DHTPIN, DHTTYPE); 

void setup() {
  //initialize sensor
  pinMode(CDSPIN, INPUT);
  pinMode(pingPin, OUTPUT);
  dht.begin();

  // initialize serial for debugging
  Serial.begin(SERIAL_BAUDRATE);
  // initialize serial for WizFi360 module
#if defined(ARDUINO_MEGA_2560)
  Serial1.begin(SERIAL1_BAUDRATE);
#elif defined(WIZFI360_EVB_PICO)
  Serial2.begin(SERIAL2_BAUDRATE);
#endif
  // initialize WizFi360 module
#if defined(ARDUINO_MEGA_2560)
  WiFi.init(&Serial1);
#elif defined(WIZFI360_EVB_PICO)
  WiFi.init(&Serial2);
#endif

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("You're connected to the network");
  printWifiStatus();
  thingspeakTrans();
}

void loop() {
  // if there's incoming data from the net connection send it out the serial port
  // this is for debugging purposes only   
//  while (client.available()) {
//    char c = client.read();
//    Serial.print("recv data: ");
//    Serial.write(c);
//    Serial.println();
//  }
  
  // if 30 seconds have passed since your last connection,
  // then connect again and send data
  if (millis() - lastConnectionTime > postingInterval) {
    //sensorRead();
    thingspeakTrans();
  }
}


//Transmitting sensor value to thingspeak
void thingspeakTrans() {
  
  // close any connection before send a new request
  // this will free the socket on the WiFi shield
  client.stop();

  // if there's a successful connection
  if (client.connect(server, 80)) {
    Serial.println("Connecting...");
    Serial.println(readUltra());
    client.print(F("GET /update?api_key="));
    client.print(apiKey);
    client.print(F("&field1="));
    client.print(readUltra());
    client.println();
    lastConnectionTime = millis();
  }
  else {
    // if you couldn't make a connection
    Serial.println("Connection failed");
  }
}

void printWifiStatus() {
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
    //IPAddress ip = WiFi.localIP();
  Serial.println(WiFi.SSID());
 Serial.println("STOPED HERE  .... ");
  // print your WiFi shield's IP address
  //IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  //Serial.println(ip);

  // print the received signal strength
  //long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  //Serial.print(rssi);
  Serial.println(" dBm");
}


long readUltra(){
  
   long duration;
   
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   pinMode(echoPin, INPUT);
 
   duration = pulseIn(echoPin, HIGH);
   Serial.print(microsecondsToInches(duration));
   //Serial.print("in, ");
   //Serial.print(microsecondsToCentimeters(duration));
   //Serial.print("cm");
   //Serial.println();
   //delay(100);

   return (microsecondsToCentimeters(duration));
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
