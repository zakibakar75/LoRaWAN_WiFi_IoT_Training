#include "WiFiEsp.h"
#include <PubSubClient.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>         

SoftwareSerial sSerial(11, 10);     // RX, TX  - Name the software serial library sftSerial (this cannot be omitted)
                                    // assigned to pins 10 and 11 for maximum compatibility

SoftwareSerial Serial1(2,3);       // For WiFi Shield

WiFiEspClient client;                 // instance of WiFi ESP Client
PubSubClient mqttClient(client);      // PubSub using the WiFI ESP Client

const char* server = "tracker.my";    // MQTT server (of your choice)
char ssid[] = "MyRouter";           // your network SSID (name)
char pass[] = "password";        // your network password
int status = WL_IDLE_STATUS;          // the Wifi radio's status

#define NUM_CIRCUITS 4              // <-- CHANGE THIS | set how many UART circuits are attached to the Tentacle

#define baud_host 115200              // set baud rate for host serial monitor(pc/mac/other)
const unsigned int send_readings_every = 20000; // set at what intervals the readings are sent to the computer (NOTE: this is not the frequency of taking the readings!)
unsigned long next_serial_time;

#define baud_circuits 9600         // NOTE: older circuit versions have a fixed baudrate (e.g. 38400. pick a baudrate that all your circuits understand and configure them accordingly)
int s0 = 7;                         // Tentacle uses pin 7 for multiplexer control S0
int s1 = 6;                         // Tentacle uses pin 6 for multiplexer control S1
int enable_1 = 5;                // Tentacle uses pin 5 to control pin E on shield 1
int enable_2 = 4;                // Tentacle uses pin 4 to control pin E on shield 2
char sensordata[30];                          // A 30 byte character array to hold incoming data from the sensors
byte sensor_bytes_received = 0;               // We need to know how many characters bytes have been received
byte code = 0;                                // used to hold the I2C response code.
byte in_char = 0;                             // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

char *channel_names[] = {"PH", "RTD", "DO", "EC"};   // <-- CHANGE THIS. A list of channel names (this list should have TOTAL_CIRCUITS entries)
                                                     // only used to designate the readings in serial communications
                                                     // position in array defines the serial channel. e.g. channel_names[0] is channel 0 on the shield; "PH" in this case.
String readings[NUM_CIRCUITS];                // an array of strings to hold the readings of each channel
int channel = 0;                              // INT pointer to hold the current position in the channel_ids/channel_names array

const unsigned int reading_delay = 200;       // delay between each reading.
                                              // low values give fast reading updates, <1 sec per circuit.
                                              // high values give your Ardino more time for other stuff
unsigned long next_reading_time;              // holds the time when the next reading should be ready from the circuit
boolean request_pending = false;              // wether or not we're waiting for a reading

const unsigned int blink_frequency = 500;     // the frequency of the led blinking, in milliseconds
unsigned long next_blink_time;                // holds the next time the led should change state
boolean led_state = LOW;                      // keeps track of the current led state

/********** For any JSON packet creation ************/
long lastMsg = 0;
char msg[100];
/************* end for JSON packet ******************/

void setup() {

  pinMode(s0, OUTPUT);                        // set the digital output pins for the serial multiplexer
  pinMode(s1, OUTPUT);
  pinMode(enable_1, OUTPUT);
  pinMode(enable_2, OUTPUT);
  pinMode(13, OUTPUT);

  sSerial.begin(baud_circuits);              // Set the soft serial port to 9600 (change if all your devices use another baudrate)
  next_serial_time = millis() + send_readings_every;  // calculate the next point in time we should do serial communications
  next_reading_time = millis() + reading_delay;
  Serial.println("-----");

  Serial.begin(baud_host);                   // Set the hardware serial port to 9600

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
  
  delay(2000);
  
  mqttClient.setServer(server, 1883);
  //mqttClient.setCallback(callback);
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
    digitalWrite(13, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(13, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void loop() {

  if (!mqttClient.connected()) {
    reconnect();
  }
  
  delay(1000);
  mqttClient.loop(); 

  sSerial.listen();
  do_sensor_readings();
  Serial1.listen();
  do_serial();

  blink_led();
}



void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.println();
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (mqttClient.connect(NULL, "myusername", "mypassword")) {
      Serial.println(F("connected"));
      // Once connected, publish an announcement...
      mqttClient.publish("watersensor1", "hello world");
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

// blinks a led on pin 13 asynchronously 
void blink_led() {
  if (millis() >= next_blink_time) {                  // is it time for the blink already?
    led_state = !led_state;                           // toggle led state on/off
    digitalWrite(13, led_state);                      // write the led state to the led pin
    next_blink_time = millis() + blink_frequency;     // calculate the next time a blink is due
  }
}



// do serial communication in a "asynchronous" way
void do_serial() {

  StaticJsonBuffer<50> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  if (millis() >= next_serial_time) {                // is it time for the next serial communication?
    Serial.println("-------------");
    for (int i = 0; i < NUM_CIRCUITS; i++) {         // loop through all the sensors
      Serial.print(channel_names[i]);                // print channel name      
      Serial.print(":\t");
      Serial.println(readings[i]);                   // print the actual reading

      if (i==0)
          root["PH"] = readings[i];

      if (i==1)
          root["RTD"] = readings[i];

      if (i==2)
          root["DO"] = readings[i];

      if (i==3)
          root["EC"] = readings[i];
          
    }
    Serial.println("-");

    root.printTo(msg);
    Serial.println();
    Serial.println(F("Publish message : "));
    Serial.println(msg);
    mqttClient.publish("watersensor1", msg);
    
    next_serial_time = millis() + send_readings_every;
  }
}


// take sensor readings in a "asynchronous" way
void do_sensor_readings() {

  if (request_pending) {                          // is a request pending?

    while (sSerial.available()) {                 // while there is data available from the circuit

      char c = sSerial.read();                    // read the next available byte from the circuit

      if (c=='\r') {                              // in case it's a <CR> character, we reached the end of a message

        sensordata[sensor_bytes_received] = 0;    // terminate the string with a \0 character
        readings[channel] = sensordata;           // update the readings array with this circuits data

        // un-comment to see the real update frequency of the readings / debug
        //Serial.print(channel_names[channel]);
        //Serial.print(" update:\t");
        //Serial.println(readings[channel]);

        sensor_bytes_received = 0;                  // reset data counter
        memset(sensordata, 0, sizeof(sensordata));  // clear sensordata array;

        request_pending = false;                    // toggle request_pending
        next_reading_time = millis()+reading_delay; // schedule the reading of the next sensor
        break;                                      // get out of this while loop, we have our data and don't care about the rest in the buffer

      } else {
        sensordata[sensor_bytes_received] = c;
        sensor_bytes_received++;
      }

    } // end while

  } else {                                     // no request is pending,
    if (millis()>next_reading_time) {
      switch_channel();                        // switch to the next channel
      request_reading();                       // do the actual UART communication
    } 
  }
}


void switch_channel() {
  channel = (channel + 1) % NUM_CIRCUITS;     // switch to the next channel (increase current channel by 1, and roll over if we're at the last channel using the % modulo operator) 
  open_channel();                             // configure the multiplexer for the new channel - we "hot swap" the circuit connected to the softSerial pins
  sSerial.flush();                            // clear out everything that is in the buffer already
}



// Request a reading from the current channel
void request_reading() {
    //sSerial.listen();
    request_pending = true;
    sSerial.print("r\r");                     // <CR> carriage return to terminate message
}


// Open a channel via the Tentacle serial multiplexer
void open_channel() {
  //sSerial.listen();
  switch (channel) {

    case 0:                                  // if channel==0 then we open channel 0
      digitalWrite(enable_1, LOW);           // setting enable_1 to low activates primary channels: 0,1,2,3
      digitalWrite(enable_2, HIGH);          // setting enable_2 to high deactivates secondary channels: 4,5,6,7
      digitalWrite(s0, LOW);                 // S0 and S1 control what channel opens
      digitalWrite(s1, LOW);                 // S0 and S1 control what channel opens
      break;

    case 1:
      digitalWrite(enable_1, LOW);
      digitalWrite(enable_2, HIGH);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, LOW);
      break;

    case 2:
      digitalWrite(enable_1, LOW);
      digitalWrite(enable_2, HIGH);
      digitalWrite(s0, LOW);
      digitalWrite(s1, HIGH);
      break;

    case 3:
      digitalWrite(enable_1, LOW);
      digitalWrite(enable_2, HIGH);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, HIGH);
      break;

    case 4:
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, LOW);
      digitalWrite(s1, LOW);
      break;

    case 5:
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, LOW);
      break;

    case '6':
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, LOW);
      digitalWrite(s1, HIGH);
      break;

    case 7:
      digitalWrite(enable_1, HIGH);
      digitalWrite(enable_2, LOW);
      digitalWrite(s0, HIGH);
      digitalWrite(s1, HIGH);
      break;
  }
}
