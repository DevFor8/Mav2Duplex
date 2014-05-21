/*
	 Mav2DupEx Version 0.1
	 2014, by DevFor8.com, info@devfor8.com
	 
	 part of code is based on ArduCAM OSD
*/

#include <SoftwareSerialO2.h>
#include <FastSerial.h>
#include <GCS_MAVLink.h>
#include <JETI_EX_SENSOR.h>
//#include <SimpleTimer.h>

#include "Vars.h"
#include "Func.h"

#define GETCHAR_TIMEOUT_ms 20  // 20ms timeout for Getchar Routine, just to make sure. Never ran into timeout

#ifndef JETI_RX
#define JETI_RX 9
#endif

#ifndef JETI_TX
#define JETI_TX 10
#endif

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port

//config data
#define ITEMNAME_1 "Armed"
#define ITEMTYPE_1 ""
#define ITEMVAL_1 (short*)&motor_armed

#define ITEMNAME_2 "Batt %"
#define ITEMTYPE_2 "%"
#define ITEMVAL_2 (short*)&osd_battery_remaining_A

#define ITEMNAME_3 "Batt V"
#define ITEMTYPE_3 "V"
#define ITEMVAL_3 &osd_vbat_A

#define ITEMNAME_4 "Batt A"
#define ITEMTYPE_4 "A"
#define ITEMVAL_4 (short*)&osd_curr_A

#define ITEMNAME_5 "Batt C"
#define ITEMTYPE_5 "mAh"
#define ITEMVAL_5 (short*)&osd_capacity_mA

#define ITEMNAME_6 "Yaw"
#define ITEMTYPE_6 "Deg"
#define ITEMVAL_6 (short*)&osd_yaw

#define ITEMNAME_7 "Pitch"
#define ITEMTYPE_7 "Deg"
#define ITEMVAL_7 (short*)&osd_pitch

#define ITEMNAME_8 "Roll"
#define ITEMTYPE_8 "Deg"
#define ITEMVAL_8 (short*)&osd_roll

#define ITEMNAME_9 "Alt"
#define ITEMTYPE_9 "m"
#define ITEMVAL_9 &osd_alt

#define ITEMNAME_10 "Home Dir"
#define ITEMTYPE_10 "Deg"
#define ITEMVAL_10 (short*)&osd_home_heading

#define ITEMNAME_11 "Home Dist"
#define ITEMTYPE_11 "m"
#define ITEMVAL_11 (short*)&osd_home_distance

#define ITEMNAME_12 "Home Alt"
#define ITEMTYPE_12 "m"
#define ITEMVAL_12 (short*)&osd_home_altdif

#define ITEMNAME_13 "GPS Lock"
#define ITEMTYPE_13 ""
#define ITEMVAL_13 (short*)&osd_fix_type_jeti

#define ITEMNAME_14 "GPS Sat"
#define ITEMTYPE_14 ""
#define ITEMVAL_14 (short*)&osd_satellites_visible

#define ITEMNAME_15 "Mode"
#define ITEMTYPE_15 ""
#define ITEMVAL_15 (short*)&base_mode



FastSerialPort0(Serial);
SoftwareSerial JetiSerial(JETI_RX,JETI_TX);
//SimpleTimer  mavlinkTimer;

void JetiUartInit() 
{
    JetiSerial.begin(9600);
}

// Transmits one byte to the box, specify if bit 9 will be set or not 
void JetiTransmitByte(unsigned char data, boolean setBit9)
{
	JetiSerial.set9bit = setBit9;
	JetiSerial.write(data);
	JetiSerial.set9bit = 0;
}

// Read the ack from the box
unsigned char JetiGetChar(void)
{
  unsigned long time = millis();
  // Wait for data to be received
  while ( JetiSerial.available()  == 0 )
  {
    if (millis()-time >  GETCHAR_TIMEOUT_ms) 
      return 0; // return, if timout occures
  }
  
  int read = -1;
  
  if (JetiSerial.available() >0 )
	read = JetiSerial.read();
  
  long wait = (millis()-time) - GETCHAR_TIMEOUT_ms;

  if (wait > 0)
  delay(wait);
  
  return read;
}

/*class JETIBoxSender : public JETI_Box_class
{
  public:
  void sendbyte(unsigned char data, bool setBit9)
  {
    JetiTransmitByte(data, setBit9);
  }
} JB;
*/

JETI_Box_class JB;

unsigned char SendFrame()
{
    for (int i = 0 ; i<JB.frameSize ; i++ )
    {
      JetiTransmitByte(JB.frame[i], JB.bit9[i]);
    }

}

unsigned char DisplayFrame()
{
    for (int i = 0 ; i<JB.frameSize ; i++ )
    {
       Serial.print(JB.frame[i],HEX);
    }
    Serial.println("");
}

short zero_val = 0;
uint8_t frame[10];
short cmpt = 0;
short value = 27;

void setup() 
{
    pinMode(13, OUTPUT);     
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    Serial.begin(TELEMETRY_SPEED);
    Serial.println("Ready");
    
    // setup mavlink port
    mavlink_comm_0_port = &Serial;
    JetiUartInit();          // Requires JETIBOX.INO library...
    JB.JetiBox("MavToDuplex","0.1");
    JB.Init("Mav2Dup");
    JB.addData(ITEMNAME_1,ITEMTYPE_1);
    JB.addData(ITEMNAME_2,ITEMTYPE_2);
    JB.addData(ITEMNAME_3,ITEMTYPE_3);
    JB.addData(ITEMNAME_4,ITEMTYPE_4);
    JB.addData(ITEMNAME_5,ITEMTYPE_5);
    JB.addData(ITEMNAME_6,ITEMTYPE_6);
    JB.addData(ITEMNAME_7,ITEMTYPE_7);
    JB.addData(ITEMNAME_8,ITEMTYPE_8);
    JB.addData(ITEMNAME_9,ITEMTYPE_9);
    JB.addData(ITEMNAME_10,ITEMTYPE_10);
    JB.addData(ITEMNAME_11,ITEMTYPE_11);
    JB.addData(ITEMNAME_12,ITEMTYPE_12);
    JB.addData(ITEMNAME_13,ITEMTYPE_13);
    JB.addData(ITEMNAME_14,ITEMTYPE_14);
    JB.addData(ITEMNAME_15,ITEMTYPE_15);
    
    //values are linked so we don't need setValue during every change but one time is enoug
    
    JB.setValue(1,ITEMVAL_1);
    JB.setValue(2,ITEMVAL_2);
    JB.setValue(3,ITEMVAL_3,1);
    JB.setValue(4,ITEMVAL_4);
    JB.setValue(5,ITEMVAL_5);
    JB.setValue(6,ITEMVAL_6);
    JB.setValue(7,ITEMVAL_7);
    JB.setValue(8,ITEMVAL_8);
    JB.setValue(9,ITEMVAL_9,1);
    JB.setValue(10,ITEMVAL_10);
    JB.setValue(11,ITEMVAL_11);
    JB.setValue(12,ITEMVAL_12);
    JB.setValue(13,ITEMVAL_13);
    JB.setValue(14,ITEMVAL_14);
    JB.setValue(15,ITEMVAL_15);
    
    do {
      JB.createFrame(1);
      SendFrame();
      delay(GETCHAR_TIMEOUT_ms);
      Serial.write(sensorFrameName);
    }
    while (sensorFrameName != 0);
    Serial.print("Ready done");

    digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
       
 }

int header = 0;

void loop() 
{

  SendFrame();
    unsigned long time = millis();

    read_mavlink();
    
    if (mavbeat == true)
      {
        digitalWrite(13, HIGH); 
      }

    
    setHomeVars();   // calculate and set Distance from home and Direction to home


    // prepare frame
    header++;
    if (header >= 45)
    {
      JB.createFrame(1);
      if (sensorFrameName == 0)
        { 
          header = 0;
       }

    }
    else
    {
      JB.createFrame(0);
    }
    // wait rest of time to 20ms
    long wait = GETCHAR_TIMEOUT_ms - (millis()-time);

    if (wait > 0)
      delay(wait);
}  
