//Code for WiFi MQTT
//Zaki, 13 August 2018
//Optimized to lower memory usage

#include "WiFiEsp.h"
#include <PubSubClient.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ArduinoJson.h>
#include "SoftwareSerial.h"

#define BUILTIN_LED 13

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2,3); // RX, TX
#endif

WiFiEspClient client;                 // instance of WiFi ESP Client
PubSubClient mqttClient(client);      // PubSub using the WiFI ESP Client

const char* server = "tracker.my";    // MQTT server (of your choice)
char ssid[] = "MyRouter10";           // your network SSID (name)
char pass[] = "xxxxxxxxxxxxxxxx";        // your network password
int status = WL_IDLE_STATUS;          // the Wifi radio's status

/********** For any JSON packet creation ************/
long lastMsg = 0;
char msg[100];
char temp[20], temp2[20], temp3[20];
int value = 0;
/************* end for JSON packet ******************/

/************ global for voltage sensor *************/
volatile float averageVcc = 0.0;
/****************************************************/

void setup()
{
  // initialize serial for debugging
  Serial.begin(115200);
  // initialize serial for ESP module
  Serial1.begin(9600);
  // initialize ESP module
  WiFi.init(&Serial1);

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("WiFi shield not present"));
    // don't continue
    while (true);
  }

  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to WPA SSID: "));
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  Serial.println(F("You're connected to the network"));

  Serial.println();
  printCurrentNet();
  printWifiData();
  
  delay(2000);

  mqttClient.setServer(server, 1883);
  //mqttClient.setCallback(callback);
  
}

void loop()
{
  StaticJsonBuffer<50> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  if (!mqttClient.connected()) {
    reconnect();
  }

  delay(1000);
  mqttClient.loop();  

  /*
  /******* Voltage Sensor ***************/
  dtostrf( (float) readVcc(), 3, 0, temp );
  root["Voltage"] = temp;
  /**************************************/

  /************** For Internal Temperature ************/
  dtostrf( 0, 3, 2, temp2 );  //Humidity set to 0.00
  root["Humidity"] = temp2;

  dtostrf( (float) GetTemp(), 3, 2, temp3 );
  root["Temperature"] = temp3;
  /****************************************************/

  root.printTo(msg);

  Serial.println();
  Serial.println(F("Publish message : "));
  Serial.println(msg);
  mqttClient.publish("UNO1", msg);
  
}

void printWifiData()
{
  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);

  // print your MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print(F("MAC address: "));
  Serial.println(buf);
}

void printCurrentNet()
{
  // print the SSID of the network you're attached to
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to
  byte bssid[6];
  WiFi.BSSID(bssid);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[5], bssid[4], bssid[3], bssid[2], bssid[1], bssid[0]);
  Serial.print(F("BSSID: "));
  Serial.println(buf);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print(F("Signal strength (RSSI): "));
  Serial.println(rssi);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print(F("Message arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}


void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.println();
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (mqttClient.connect(NULL, "zaki.bm@gmail.com", "xxxxxxxx")) {
      Serial.println(F("connected"));
      // Once connected, publish an announcement...
      mqttClient.publish("UNO1", "hello world");
      // ... and resubscribe
      //mqttClient.subscribe("UNO1Rx");
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(mqttClient.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

double GetTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.
  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 304.31 ) / 1.22;

  // The returned temperature is in degrees Celcius.
  return (t);
}


