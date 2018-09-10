#include "WiFiEsp.h"
#include <ArduinoJson.h>

// Emulate Serial1 on pins 6/7 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(2, 3); // RX, TX
#endif

char ssid[] = "MyRouter";            // your network SSID (name)
char pass[] = "xxxxxxxxxxxxx";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

char server[] = "xxxxxxx.fred.sensetecnic.com";

// Initialize the Ethernet client object
WiFiEspClient client;

/********** For any JSON packet creation ************/
char msg[100];
/************* end for JSON packet ******************/

unsigned long previousMillis = 0; 
const long interval = 10000;           // interval for upload

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

  // you're connected now, so print out the data
  Serial.println("You're connected to the network");

  printWifiStatus();
  Serial.println();
  
}

void loop()
{
  StaticJsonBuffer<50> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  int lines_received = 0;
  String json;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) 
  {
    // save the last time you sent data
    previousMillis = currentMillis;
  
    root["Humidity"] = 30;  //just hardcode a value
    root["Temperature"] = 60;  //just hardcode a value
    root.printTo(msg);
 
    Serial.println();
    Serial.println("Starting connection to server...");  
    
    // if you get a connection, report back via serial
    if (client.connect(server,80)) {
        Serial.println("Connected to server");
        // Make a HTTP request
        String content = msg;
        client.println("POST /api/public/zaki HTTP/1.1");
        client.println("Host: xxxxxxxx.fred.sensetecnic.com");
        client.println("Accept: */*");
        client.println("Content-Length: " + String(content.length()));
        client.println("Content-Type: application/json");
        client.println();
        client.println(content);
    }

    // if there are incoming bytes available from the server, read them and print them
    while (client.available()) {
        Serial.println("Data received from server...");
        String line = client.readStringUntil('\r\n');
        if (lines_received > 0) { 
            json = line;  
        }
        lines_received++;

        if(json.length() > 0){
            Serial.println("from server: "+json);
            JsonObject& root = jsonBuffer.parseObject(json);
            root.prettyPrintTo(Serial);
        }
    }

    Serial.println();
    Serial.println("Disconnecting from server...");
    client.stop();

  }
}


void printWifiStatus()
{
  // print the SSID of the network you're attached to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}