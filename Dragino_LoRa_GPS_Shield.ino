/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the (early prototype version of) The Things Network.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1,
 *  0.1% in g2).
 *
 * Change DEVADDR to a unique address!
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 * 29 December 2016 Modified by Zaki to cater for RF95 + Arduino 
 *******************************************************************************/

#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>

#include <TinyGPS.h>           //For GPS
#include <SoftwareSerial.h>    //For GPS

TinyGPS gps;                   // Instantiate GPS object
SoftwareSerial ss(3, 4);       // Arduino RX, TX to conenct to GPS module.

/********* For GPS Stuff **********************/
static void smartdelay(unsigned long ms);

unsigned int count = 1;        //For times count

uint8_t datasend[6];     //Used to store GPS data for uploading

char gps_lon[15]={"\0"};  //Storage GPS info
char gps_lat[15]={"\0"}; //Storage latitude
//char gps_alt[15]={"\0"}; //Storage altitude

float flat, flon;
//float falt;
/**********************************************/
                  
//volatile float averageVcc = 0.0;  // for voltage sensor


/************************************* OTAA Section : must fill these ******************************************************************/
/* This EUI must be in little-endian format, so least-significant-byte
   first. When copying an EUI from ttnctl output, this means to reverse
   the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3, 0x70. */
   
//static const u1_t PROGMEM APPEUI[8]={ 0x5E, 0x1D, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };
/*Eg : 70B3D57EF0001D5E as got from TTN dashboard*/
//void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

/* This should also be in little endian format, see above. */
//static const u1_t PROGMEM DEVEUI[8]={ 0x69, 0x32, 0x30, 0x89, 0x05, 0xE4, 0x54, 0x00 };
/*Eg : 0054E40589303269 as got from TTN dashboard */
//void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

/* This key should be in big endian format (or, since it is not really a
   number but a block of memory, endianness does not really apply). In
   practice, a key taken from ttnctl can be copied as-is.
   The key shown here is the semtech default key. */
//static const u1_t PROGMEM APPKEY[16] = { 0xFF, 0xDF, 0x9C, 0x9A, 0x03, 0x55, 0xFF, 0xAC, 0xA3, 0xD7, 0xDA, 0x52, 0x02, 0xC8, 0xFC, 0xE2 };
/*Eg : FFDF9C9A0355FFACA3D7DA5202C8FCE2 as got from TTN dashboard */
//void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

/**************************************** OTAA Section Ends *********************************************************************************/


/********************************* ABP Section : Need to fill these *************************************************************************/
/* LoRaWAN NwkSKey, network session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
static const PROGMEM u1_t NWKSKEY[16] = { 0x5C, 0x92, 0x4D, 0x44, 0xCB, 0x15, 0xE9, 0x35, 0xA6, 0x60, 0x48, 0xD6, 0xA7, 0x29, 0x16, 0xAF };
/*Eg : 9FE76F0216C06C4A3449DE5C9AF56162 as got from TTN dashboard*/

/* LoRaWAN AppSKey, application session key
   This is the default Semtech key, which is used by the prototype TTN
   network initially. */
static const u1_t PROGMEM APPSKEY[16] = { 0x29, 0x09, 0x03, 0x72, 0x7E, 0x57, 0x3A, 0xC7, 0x61, 0x74, 0xA4, 0x4E, 0x85, 0xB2, 0xF3, 0x1D };
/*Eg : 6CA021BC3EF5A903D0CFD65557291CA0 as got from TTN dashboard */

/* LoRaWAN end-device address (DevAddr)
   See http://thethingsnetwork.org/wiki/AddressSpace */
static const u4_t DEVADDR = 0x26021AE6 ; // <-- Change this address for every node!
/*Eg : 26021F2E as got from TTN dashboard */

/* These callbacks are only used in over-the-air activation, so they are
   left empty here (we cannot leave them out completely unless
   DISABLE_JOIN is set in config.h, otherwise the linker will complain). */
   
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

/********************************************* ABP Section Ends ***************************************************************************/

static osjob_t sendjob;

/* Schedule TX every this many seconds (might become longer due to duty
   cycle limitations). */
const unsigned TX_INTERVAL = 10;

/* Pin mapping */
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                 // data received in rx slot after tx
                 Serial.print(F("Received "));
                 Serial.print(LMIC.dataLen);
                 Serial.print(F(" bytes of payload: 0x"));
                 for (int i = 0; i < LMIC.dataLen; i++) {
                    if (LMIC.frame[LMIC.dataBeg + i] < 0x10) 
                    {
                        Serial.print(F("0"));
                    }
                    Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                    
                    if (i==0) //check the first byte
                    {
                      if (LMIC.frame[LMIC.dataBeg + 0] == 0x00)
                      {
                          Serial.print(F(" Yes!!!! "));
                      }
                    }
                    
                 }
                 Serial.println();
            }
            
            /* Schedule next transmission */
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            /* data received in ping slot */
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    
    //static uint8_t message[6];
    
    /* Check if there is not a current TX/RX job running */
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        /* Prepare upstream data transmission at the next possible time. */

        Serial.println(" ");  
        
        /******* Voltage Sensor ***************/
        //averageVcc = averageVcc + (float) readVcc();
        //int16_t averageVccInt = round(averageVcc);
        //Serial.print(averageVccInt);
        //Serial.print(" mV \n"); 
        //message[0] = highByte(averageVccInt);
        //message[1] = lowByte(averageVccInt);
        /**************************************/

        /************** For Internal Temperature ************/
        //Serial.println(F("Internal Temperature Sensor"));
        //Serial.println(GetTemp(),1);
        //delay(1000);
        //int16_t averageIntTemp = GetTemp() * 100;
        //Serial.println(averageIntTemp);
        //No humidity sensor here, just put 0 for byte 2 and 3
        //message[2] = 0;
        //message[3] = 0;
        //message[4] = highByte(averageIntTemp);
        //message[5] = lowByte(averageIntTemp);
        /****************************************************/

        /************** For GPS Data ************************/
        GPSRead();
        GPSWrite();
        /****************************************************/

        LMIC_setTxData2(1, datasend, sizeof(datasend), 0);     //for sending GPS
        //LMIC_setTxData2(1, message, sizeof(message), 0);    //for sending Voltage, Temp, Humidity   
        Serial.println(F("Packet queued"));
        /*Print Freq being used*/
        Serial.print("Transmit on Channel : ");Serial.println(LMIC.txChnl);
        /*Print mV being supplied*/
        
        delay(1000);
        //averageVcc = 0;
                
    }
    /* Next TX is scheduled after TX_COMPLETE event. */
}

void setup() {
    Serial.begin(115200);   // for Serial Monitor
    ss.begin(9600);         // for GPS Serial Comms
    
    Serial.println(F("Starting"));   

    LoraInitialization();  // Do all Lora Init Stuff

    /* Start job */
    do_send(&sendjob);
}

void loop() {   
    os_runloop_once();   
}

/*
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
*/

void LoraInitialization(){
    /* LMIC init */
    os_init();
    /* Reset the MAC state. Session and pending data transfers will be discarded. */
    LMIC_reset();

    /****************** ABP Only uses this section *****************************************/
    /* Set static session parameters. Instead of dynamically establishing a session
       by joining the network, precomputed session parameters are be provided.*/
    #ifdef PROGMEM
    /* On AVR, these values are stored in flash and only copied to RAM
       once. Copy them to a temporary buffer here, LMIC_setSession will
       copy them into a buffer of its own again. */
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    /* If not running an AVR with PROGMEM, just use the arrays directly */
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    /******************* ABP Only Ends ****************************************************/

    /* Disable link check validation */
    LMIC_setLinkCheckMode(0);

    /* TTN uses SF9 for its RX2 window */
    LMIC.dn2Dr = DR_SF9;
    
    /* Set data rate and transmit power (note: txpow seems to be ignored by the library) */
    LMIC_setDrTxpow(DR_SF12,20);  //lowest Datarate possible in 915MHz region

}


void GPSRead()
{
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    //falt=gps.f_altitude();  //get altitude    
    flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6;   
    flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6;//save six decimal places 
    //falt == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : falt, 2;//save two decimal places 
}

void GPSWrite()
{
    /*Convert GPS data to format*/
    dtostrf(flat, 0, 4, gps_lat);   
    dtostrf(flon, 0, 4, gps_lon);
  
    if(flon!=1000.000000)
    {  
      strcat(gps_lon, ",");
      strcat(gps_lon, gps_lat); 
              
      atof(gps_lon);  
      Serial.println(F("Testing converted data:"));
      Serial.println(gps_lon);
  
      strcpy(datasend, gps_lon); //the format of datasend is longtitude,latitude,altitude
  
      Serial.print(F("###########    "));
      Serial.print(F("NO."));
      Serial.print(count);
      Serial.println(F("    ###########"));
      Serial.println(F("The longtitude and latitude are:"));
      Serial.print(F("["));
      Serial.print((char*)datasend);
      Serial.print(F("]"));
      Serial.print(F(""));

      count++;
    }
  
    int32_t lng = flat * 10000;   //eliminate the decimal point 
    int32_t lat = flon * 10000;   //eliminate the decimal point

    datasend[0] = lat;
    datasend[1] = lat >> 8;
    datasend[2] = lat >> 16;

    datasend[3] = lng;
    datasend[4] = lng >> 8;
    datasend[5] = lng >> 16;
    
    smartdelay(1000);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
    {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}
