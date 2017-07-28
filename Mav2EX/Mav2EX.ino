//#define DEBUG

#ifndef DEBUG
  //disable stock serial driver
  #include "disableserial.h" 
#endif

#include <FastSerial.h>
#include <Arduino.h>

#include <EEPROM.h>

/*
	 Mav2DupEx Version 1.0
	 2014, by DevFor8.com, info@devfor8.com

	 part of code is based on ArduCAM OSD
         Bugfixes and some enhancements added by MTBSTEVE

   2017, by Radek Voltr, voltr@voltr.eu, version 2.0
   memory optimization, new jeti sensor wire up, fastserial removal, Mega32u4 compatibility

*/

#include "SoftwareSerialO2.h"
#include "JETI_EX_SENSOR.h"

#include <GCS_MAVLink.h>
#define MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN 10

#include "Vars.h"
#include "Func.h"

#define GETCHAR_TIMEOUT_ms 100  // 100ms instead of 20ms to let code time to hit fastserial readings

#ifndef JETI_RX
#define JETI_RX 10
#endif

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port, 57600 for sure, 115200 better

#define ABOUT_1 F("MavToEX 2.0")
#define ABOUT_2 F("Nav with < >")

const unsigned char Jeti_SensorID3 = 0x02;
const unsigned char Jeti_SensorID4 = 0x02;

char * floatToString(char * outstr, float value, int places, int minwidth = 0) {
  // this is used to write a float value to string, outstr.  oustr is also the return value.
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;
  int c = 0;
  int charcount = 1;
  int extra = 0;
  // make sure we round properly. this could use pow from <math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as 54.3209

  // calculate rounding term d:   0.5/pow(10,places)
  float d = 0.5;
  if (value < 0)
    d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
    d /= 10.0;
  // this small addition, combined with truncation will round our values properly
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  if (value < 0)
    tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) {
    tens *= 10.0;
    tenscount += 1;
  }

  if (tenscount > 0)
    charcount += tenscount;
  else
    charcount += 1;

  if (value < 0)
    charcount += 1;
  charcount += 1 + places;

  minwidth += 1; // both count the null final character
  if (minwidth > charcount) {
    extra = minwidth - charcount;
    charcount = minwidth;
  }

  // write out the negative if needed
  if (value < 0)
    outstr[c++] = '-';

  if (tenscount == 0)
    outstr[c++] = '0';

  for (i = 0; i < tenscount; i++) {
    digit = (int) (tempfloat / tens);
    itoa(digit, &outstr[c++], 10);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return

  // otherwise, write the point and continue on
  if (places > 0)
    outstr[c++] = '.';


  // now write out each decimal place by shifting digits one by one into the ones place and writing the truncated value
  for (i = 0; i < places; i++) {
    tempfloat *= 10.0;
    digit = (int) tempfloat;
    itoa(digit, &outstr[c++], 10);
    // once written, subtract off that digit
    tempfloat = tempfloat - (float) digit;
  
    
  }
  if (extra > 0 ) {
    for (int i = 0; i < extra; i++) {
      outstr[c++] = ' ';
    }
  }


  outstr[c++] = '\0';

  return outstr;
}

JETI_Box_class JB;


short zero_val = 0;
uint8_t frame[10];
short cmpt = 0;
short value = 27;

#define MAX_SCREEN 6
#define MAX_CONFIG 4
#define COND_LES_EQUAL 1
#define COND_MORE_EQUAL 2

struct Alarm
{
  const __FlashStringHelper* name;
  const __FlashStringHelper* type;
  int limit_max;
  int limit_min;
  float val_step;
  byte condition;
  byte current_set;
  char alarm;
  byte reset_alarm;
};

Alarm Alarms[MAX_CONFIG];

void InitAlarms()
{
  Alarms[0].name = F("Min Voltage");
  Alarms[0].type = F("V");
  Alarms[0].limit_max = 200;
  Alarms[0].limit_min = 1;
  Alarms[0].val_step = 0.25;
  Alarms[0].condition = COND_LES_EQUAL;
  Alarms[0].current_set = EEPROM.read(0);
  Alarms[0].alarm = 'U';
  Alarms[0].reset_alarm = 0;


  Alarms[1].name = F( "Max Capacity");
  Alarms[1].type = F( "mAh");
  Alarms[1].limit_max = 200;
  Alarms[1].limit_min = 1;
  Alarms[1].val_step = 100;
  Alarms[1].condition = COND_MORE_EQUAL;
  Alarms[1].current_set = EEPROM.read(1);
  Alarms[1].alarm = 'C';
  Alarms[1].reset_alarm = 0;

  Alarms[2].name = F( "Max Altitude");
  Alarms[2].type = F( "m");
  Alarms[2].limit_max = 200;
  Alarms[2].limit_min = 1;
  Alarms[2].val_step = 5;
  Alarms[2].condition = COND_MORE_EQUAL;
  Alarms[2].current_set = EEPROM.read(2);
  Alarms[2].alarm = 'A';
  Alarms[2].reset_alarm = 0;

  Alarms[3].name = F( "Max Distance");
  Alarms[3].type = F( "m");
  Alarms[3].limit_max = 200;
  Alarms[3].limit_min = 1;
  Alarms[3].val_step = 10;
  Alarms[3].condition = COND_MORE_EQUAL;
  Alarms[3].current_set = EEPROM.read(3);
  Alarms[3].alarm = 'D';
  Alarms[3].reset_alarm = 0;

}

void ProcessAlarm(int id, float current_val)
{
  if (Alarms[id].reset_alarm > 0)
  {
    Alarms[id].reset_alarm--;
    return;
  }

  if ((Alarms[id].condition == COND_MORE_EQUAL) && (Alarms[id].current_set != 255) )
  {
    if ((current_val > 0) && (current_val >= Alarms[id].current_set * Alarms[id].val_step))
    {
      //Serial.print("Alarm more:");Serial.print(id);Serial.print(" ");Serial.print(current_val);Serial.print(" ");Serial.print(Alarms[id].current_set * Alarms[id].val_step);Serial.print(" ");Serial.println(Alarms[id].current_set);
      JB.alarm(Alarms[id].alarm);
    }
  }
  else if ((Alarms[id].condition == COND_LES_EQUAL) && (Alarms[id].current_set != 255) )      {
    if ( (current_val > 0) && (current_val <= Alarms[id].current_set * Alarms[id].val_step) )
    {
      //Serial.print("Alarm Les:");Serial.print(id);Serial.print(" ");Serial.print(current_val);Serial.print(" ");Serial.print(Alarms[id].current_set * Alarms[id].val_step);Serial.print(" ");Serial.println(Alarms[id].current_set);
      JB.alarm(Alarms[id].alarm);
    }
  }
}

void JetiboxISR()
{
  if (mavlink_comm_0_port->available() >= MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN ) //read one MAV frame between each JB value
    read_mavlink(1);
}

int delayMAV(int _delay)
{
  int read = 0;
  unsigned long wait_till = millis() + _delay;
  while ( millis() <  wait_till )
  {
    if (mavlink_comm_0_port->available() >= MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN ) //at least beat should fill
      {
      read_mavlink(2);
      }

    if (JetiSerial.available() > 0 )
    {
      read = JB.readbuttons();
      if (read != 240 && read != 0)
      { 
        break;
      }
    }
  }
  return read;
}

#if defined(__AVR_ATmega328P__)
  FastSerialPort(FSerial,0);
#else //Mega32U4
  FastSerialPort(FSerial,1);
#endif

BetterStream  *mavlink_comm_0_port;
BetterStream  *mavlink_comm_1_port;
mavlink_system_t mavlink_system; //modified

int current_screen = 1; // 0 - about , 2 - message ,1 - prestart (arm/gps mode/gps count/ voltage/ one line message), 3- flight (mode, home angle, home alt) 4 - power state (v/cur/capacity/perc), 5 IMU status (angles, alt), 6 - Alarms
int current_config = 0; // 0 - Alarms, 1 - capacity warn, 2 - dist warn
int last_screen = 1;

    
void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)

  // setup mavlink port
#if defined(__AVR_ATmega328P__)
  
#else //Mega32U4
  
  //debug
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) {};
    Serial.println("Debug started!");
  #endif  
#endif

  FSerial.begin(TELEMETRY_SPEED,512,8);
  mavlink_comm_0_port = &FSerial;


  pinMode(JETI_RX, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(3, LOW);

  strcpy_P((char*)&LastMessage, (const char*)F("Mav2EX Init OK"));

  current_screen = EEPROM.read(MAX_CONFIG); //after alarms
  last_screen = current_screen;


  InitAlarms();


  //Serial.println(F("JetiUartInit")); //progmem

  JB.JetiBox_P(ABOUT_1, ABOUT_2); //change to this for copy directly from F() and also to AddData in F() form only
  JB.Init(F("Mav2EX"), JETI_RX, 9700);
  JB.FrameISR = JetiboxISR;

  //values are linked so we don't need setValue during every change but one time is enoug

  motor_armed = 0;

  JB.setValue6(JB.addData(F("Mode"), F("")), &osd_mode, 0);
  JB.setValue6(JB.addData(F("GPS Lock"), F("")), &osd_fix_type_jeti, 0);
  JB.setValue6(JB.addData(F("GPS Sat"), F("")), &osd_satellites_visible, 0);

  JB.setValue14(JB.addData(F("Batt %"), F("%")), &osd_battery_remaining_A, 0);
  JB.setValue14(JB.addData(F("Home Dir"), F("Deg")), &osd_home_heading, 0);
  JB.setValue14(JB.addData(F("HDOP"), F("")), &ap_gps_hdop, 2);
  /*
   JB.setValue14(JB.addData(F("Speed"), F("m/s")), &osd_groundspeed, 1);
   JB.setValue14(JB.addData(F("Climb"), F("m/s")), &osd_climb, 1);
  */
   JB.setValue14(JB.addData(F("Pitch"), F("Deg")), &osd_pitch, 0);
   JB.setValue14(JB.addData(F("Roll"), F("Deg")), &osd_roll, 0);
  
  JB.setValue14(JB.addData(F("Alt"), F("m")), &osd_alt, 0);
  JB.setValue14(JB.addData(F("Batt V"), F("V")), &osd_vbat_A, 1);
  JB.setValue14(JB.addData(F("Batt A"), F("A")), &osd_curr_A, 1);
  JB.setValue30(JB.addData(F("Batt C"), F("mAh")), &osd_capacity_mA, 0);

  //long range flights only
  //JB.setValue30(JB.addData(F("Home Dist"), F("m")), &osd_home_distance, 0);
  JB.setValue14(JB.addData(F("Home Dist"), F("m")), &osd_home_distance, 0);

  JB.setValueGPS(JB.addData( F("Lat"), F("")), &osd_lat, false);
  JB.setValueGPS(JB.addData( F("Lon"), F("")), &osd_lon, true);

  JB.SendFrame();
  delayMAV(GETCHAR_TIMEOUT_ms);

  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)

}




int header = 0;
int alarm_id = -1;
float alarm_current = 0;

void process_screens()
{
  char msg_line1[LCDMaxPos / 2];
  char msg_line2[LCDMaxPos / 2];
  char temp[LCDMaxPos / 2];

  if (last_screen != current_screen)
  //screen changed
  {
    EEPROM.write(MAX_CONFIG,current_screen); //after alarms
    last_screen = current_screen;
  }

  switch (current_screen)
  {
    case 0 : {
        JB.JetiBox_P(ABOUT_1, ABOUT_2);
        break;
      }
    case 2 : {
        JB.JetiBox(LastMessage);
        break;
      }
    case 1 : {
        msg_line1[0] = 0;      msg_line2[0] = 0;
        if (motor_armed == 1)
          strcat_P((char*)&msg_line1, (const char*)F("ARM "));
        else
          strcat_P((char*)&msg_line1, (const char*)F("DIS "));

      if (osd_fix_type_jeti == 0)
        strcat_P((char*)&msg_line1,(const char*)F("NO:")); // no Sats
        else
        if (osd_fix_type_jeti == 1)
          strcat_P((char*)&msg_line1,(const char*)F("NO:")); // <3 Sats, 1D but essentially NO Fix too
          else
          if (osd_fix_type_jeti == 2)
            strcat_P((char*)&msg_line1,(const char*)F("2D:"));  // 3-4 Sats
             else
             if (osd_fix_type_jeti >= 3)
                 strcat_P((char*)&msg_line1,(const char*)F("3D:"));  // >4 Sats, helmarw: some RTK gps may give fix type 4

        temp[0] = 0;
        int i = osd_satellites_visible;
        if (i < 0)
          i = 0;
        else if (i > 99)
          i = 99;
        itoa (i, (char*)&temp, 10);
        strcat((char*)&msg_line1, (char*)&temp);
        strcat_P((char*)&msg_line1, (const char*)F(" "));

        temp[0] = 0;
        floatToString((char*)&temp, (float)osd_vbat_A/10, 1);
        strcat((char*)&msg_line1, (char*)&temp);
        strcat_P((char*)&msg_line1, (const char*)F("V"));

       // strncpy((char*)&msg_line2, (char*)LastMessage, LCDMaxPos / 2);  helmarw: removed this since its on screen2 already, usually to long for one line anyway
	
      // helmarw: added this instead, to have a comprehensive default screen, thats the only one im using usually
	 
	   const __FlashStringHelper* mode_str=F("unkn");
     if(apm_mav_type == 1){ //ArduPlane
        if (osd_mode == 0)       mode_str = F("Manu"); //Manual
        else if (osd_mode == 1)  mode_str = F("Circ"); //Circle
        else if (osd_mode == 2)  mode_str = F("Stab"); //Stabilize
        else if (osd_mode == 3)  mode_str = F("Trng"); //Training
        else if (osd_mode == 4)  mode_str = F("Acro"); //Acro
        else if (osd_mode == 5)  mode_str = F("Fbwa"); //Fly_By_Wire_A
        else if (osd_mode == 6)  mode_str = F("Fbwb"); //Fly_By_Wire_B
        else if (osd_mode == 7)  mode_str = F("Crui"); //Cruise
        else if (osd_mode == 8)  mode_str = F("Atun"); //Auto Tune
        else if (osd_mode == 10) mode_str = F("Auto"); //Auto
        else if (osd_mode == 11) mode_str = F("RTL"); //Return to Launch
        else if (osd_mode == 12) mode_str = F("Loit"); //Loiter
        else if (osd_mode == 15) mode_str = F("Guid"); //Guided
        else if (osd_mode == 16) mode_str = F("Init"); //Initializing
    }
    else    
    { //rest
        if (osd_mode == 0)       mode_str = F("Stab"); //Stabilize: hold level position
        else if (osd_mode == 1)  mode_str = F("Acro"); //Acrobatic: rate control
        else if (osd_mode == 2)  mode_str = F("Alth"); //Altitude Hold: auto control
        else if (osd_mode == 3)  mode_str = F("Auto"); //Auto: auto control
        else if (osd_mode == 4)  mode_str = F("Guid"); //Guided: auto control
        else if (osd_mode == 5)  mode_str = F("Loit"); //Loiter: hold a single location
        else if (osd_mode == 6)  mode_str = F("RTL "); //Return to Launch: auto control
        else if (osd_mode == 7)  mode_str = F("Circ"); //Circle: auto control
        else if (osd_mode == 8)  mode_str = F("Posi"); //Position: auto control
        else if (osd_mode == 9)  mode_str = F("Land"); //Land:: auto control
        else if (osd_mode == 10) mode_str = F("Oflo"); //OF_Loiter: hold a single location using optical flow sensor
        else if (osd_mode == 11) mode_str = F("Drif"); //Drift mode: 
        else if (osd_mode == 13) mode_str = F("Sprt"); //Sport: earth frame rate control
        else if (osd_mode == 14) mode_str = F("Flip"); //Flip: flip the vehicle on the roll axis
        else if (osd_mode == 15) mode_str = F("Atun"); //Auto Tune: autotune the vehicle's roll and pitch gains
        else if (osd_mode == 16) mode_str = F("PosH"); //Hybrid: position hold with manual override
    }
    
        strcat_P((char*)&msg_line2, (const char*)mode_str);
        strcat_P((char*)&msg_line2, (const char*)F(" Alt:"));
        temp[0] = 0;
        floatToString((char*)&temp, (float)osd_alt, 0);
        strcat((char*)&msg_line2, (char*)&temp);
        strcat_P((char*)&msg_line2, (const char*)F("m"));
	
   // helmarw: end addition, please test it, for me it works very well, using it for years
	    
        JB.JetiBox((char*)&msg_line1, (char*)&msg_line2);
        break;
	    
      }
    case 3 : {

        const __FlashStringHelper* mode_str = F("unkn");
        if (apm_mav_type == 1) { //ArduPlane
          if (osd_mode == 0)       mode_str = F("Manu"); //Manual
          else if (osd_mode == 1)  mode_str = F("Circ"); //Circle
          else if (osd_mode == 2)  mode_str = F("Stab"); //Stabilize
          else if (osd_mode == 3)  mode_str = F("Trng"); //Training
          else if (osd_mode == 4)  mode_str = F("Acro"); //Acro
          else if (osd_mode == 5)  mode_str = F("Fbwa"); //Fly_By_Wire_A
          else if (osd_mode == 6)  mode_str = F("Fbwb"); //Fly_By_Wire_B
          else if (osd_mode == 7)  mode_str = F("Crui"); //Cruise
          else if (osd_mode == 8)  mode_str = F("Atun"); //Auto Tune
          else if (osd_mode == 10) mode_str = F("Auto"); //Auto
          else if (osd_mode == 11) mode_str = F("RTL"); //Return to Launch
          else if (osd_mode == 12) mode_str = F("Loit"); //Loiter
          else if (osd_mode == 15) mode_str = F("Guid"); //Guided
          else if (osd_mode == 16) mode_str = F("Init"); //Initializing
        }
        else
        { //rest
          if (osd_mode == 0)       mode_str = F("Stab"); //Stabilize: hold level position
          else if (osd_mode == 1)  mode_str = F("Acro"); //Acrobatic: rate control
          else if (osd_mode == 2)  mode_str = F("Alth"); //Altitude Hold: auto control
          else if (osd_mode == 3)  mode_str = F("Auto"); //Auto: auto control
          else if (osd_mode == 4)  mode_str = F("Guid"); //Guided: auto control
          else if (osd_mode == 5)  mode_str = F("Loit"); //Loiter: hold a single location
          else if (osd_mode == 6)  mode_str = F("RTL "); //Return to Launch: auto control
          else if (osd_mode == 7)  mode_str = F("Circ"); //Circle: auto control
          else if (osd_mode == 8)  mode_str = F("Posi"); //Position: auto control
          else if (osd_mode == 9)  mode_str = F("Land"); //Land:: auto control
          else if (osd_mode == 10) mode_str = F("Oflo"); //OF_Loiter: hold a single location using optical flow sensor
          else if (osd_mode == 11) mode_str = F("Drif"); //Drift mode:
          else if (osd_mode == 13) mode_str = F("Sprt"); //Sport: earth frame rate control
          else if (osd_mode == 14) mode_str = F("Flip"); //Flip: flip the vehicle on the roll axis
          else if (osd_mode == 15) mode_str = F("Atun"); //Auto Tune: autotune the vehicle's roll and pitch gains
          else if (osd_mode == 16) mode_str = F("PosH"); //Hybrid: position hold with manual override
        }


        msg_line1[0] = 0;      msg_line2[0] = 0;
        strcat_P((char*)&msg_line1, (const char*)mode_str);
        strcat_P((char*)&msg_line1, (const char*)F(" Alt:"));
        temp[0] = 0;
        floatToString((char*)&temp, (float)osd_alt, 0);
        strcat((char*)&msg_line1, (char*)&temp);
        strcat_P((char*)&msg_line1, (const char*)F("m"));

        strcat_P((char*)&msg_line2, (const char*)F("Dis:"));
        temp[0] = 0;
        //floatToString((char*)&temp,osd_home_distance,0);
        itoa(osd_home_distance, (char*)&temp, 10);
        strcat((char*)&msg_line2, (char*)&temp);
        strcat_P((char*)&msg_line2, (const char*)F("m Dir:"));
        itoa (osd_home_heading, (char*)&temp, 10);
        strcat((char*)&msg_line2, (char*)&temp);

        JB.JetiBox((char*)&msg_line1, (char*)&msg_line2);
        break;
      }
    case 4 : {
        msg_line1[0] = 0;      msg_line2[0] = 0;
        temp[0] = 0;
        strcat_P((char*)&msg_line1, (const char*)F("Power "));

        floatToString((char*)&temp, (float)osd_vbat_A/10, 1);
        strcat((char*)&msg_line1, (char*)&temp);
        strcat_P((char*)&msg_line1, (const char*)F("V "));

        temp[0] = 0;
        floatToString((char*)&temp, (float)osd_curr_A/10, 1);
        strcat((char*)&msg_line2, (char*)&temp);
        strcat_P((char*)&msg_line2, (const char*)F("A "));

        itoa (osd_capacity_mA, (char*)&temp, 10);
        strcat((char*)&msg_line2, (char*)&temp);
        strcat_P((char*)&msg_line2, (const char*)F("mAh"));

        JB.JetiBox((char*)&msg_line1, (char*)&msg_line2);
        break;
      }
    case 5 : {
        msg_line1[0] = 0;      msg_line2[0] = 0;
        strcat_P((char*)&msg_line1, (const char*)F("Rol:"));
        temp[0] = 0;
        itoa(osd_roll, (char*)&temp, 10);
        strcat((char*)&msg_line1, (char*)&temp);
        strcat_P((char*)&msg_line1, (const char*)F(" Yaw:"));
        itoa (osd_yaw, (char*)&temp, 10);
        strcat((char*)&msg_line1, (char*)&temp);

        strcat_P((char*)&msg_line2, (const char*)F("Pit:"));
        temp[0] = 0;
        itoa(osd_pitch, (char*)&temp, 10);
        strcat((char*)&msg_line2, (char*)&temp);
        strcat_P((char*)&msg_line2, (const char*)F(" Alt:"));
        floatToString((char*)&temp, (float)osd_alt, 0,3);
        strcat((char*)&msg_line2, (char*)&temp);

        JB.JetiBox((char*)&msg_line1, (char*)&msg_line2);
        break;
      }
    case MAX_SCREEN : {

        msg_line1[0] = 0;      msg_line2[0] = 0;

        if (alarm_id != current_config)
        {
          if (alarm_id != 0)
          {
            //store to eprom
            EEPROM.write(alarm_id - 1, Alarms[alarm_id - 1].current_set);
          }
          if (current_config != 0)
          {
            //read from array to


          }
          alarm_id = current_config;
        }


        if (current_config == 0)
        {
          strcpy_P((char*)&msg_line1, (const char*)F("Alarms"));
          strcpy_P((char*)&msg_line2, (const char*)F("Use Up/Down"));
        }
        else
        {
          // Serial.print(Alarms[alarm_id-1].name); Serial.print(" ");
          strcpy_P((char*)&msg_line1, (const char*)Alarms[alarm_id - 1].name);
          //Serial.println(msg_line1);

          if (Alarms[alarm_id - 1].current_set == 255)
            strcpy_P((char*)&msg_line2, (const char*)F("<> Not Active"));
          else
          {
            if (Alarms[alarm_id - 1].condition == COND_MORE_EQUAL)
              strcpy_P((char*)&msg_line2, (const char*)F(" > "));
            else if (Alarms[alarm_id - 1].condition == COND_LES_EQUAL)
              strcpy_P((char*)&msg_line2, (const char*)F(" < "));

            floatToString((char*)&temp, Alarms[alarm_id - 1].val_step * Alarms[alarm_id - 1].current_set , 1);
            strcat((char*)&msg_line2, (char*)&temp);
            strcat_P((char*)&msg_line2, (const char*)F(" "));
            strcat_P((char*)&msg_line2, (const char*)Alarms[alarm_id - 1].type);
          }
        }

        JB.JetiBox((char*)&msg_line1, (char*)&msg_line2);
      }
  }

  //process alarms
  ProcessAlarm(0, osd_vbat_A/10);
  ProcessAlarm(1, osd_capacity_mA);
  ProcessAlarm(2, osd_alt);
  ProcessAlarm(3, osd_home_distance);



}


void loop()
{
  /*if (mavlink_comm_0_port->available() > 0) //at least beat should fill
    read_mavlink(5);
    */

  if (current_screen != MAX_SCREEN)
    current_config = 0; //zero 5th screen

  process_screens();

  /*if (mavlink_comm_0_port->available() > 0) //at least beat should fill
    read_mavlink(5);
    */

  setHomeVars();   // calculate and set Distance from home and Direction to home

  // prepare frame
  JB.txMode();
  JB.SendFrame();

  JB.rxMode();

  int read = delayMAV (GETCHAR_TIMEOUT_ms);

  //character should came in 20ms wait cycle and one only so we don't need to break wait and calculate resting time
  if ( (read != 240 ) && (read != 0) )
  { //240 = no buttons
    //224 - right
    //112 - left
    //208 up
    //176 down
    //144 up+down
    //96 left+right

    // process buttons
    switch (read)
    {
      case 224 : if ( (current_screen  != MAX_SCREEN) || (alarm_id == 0) )
        {
          current_screen++;
          if (current_screen > MAX_SCREEN) current_screen = 0;
        }
        else
        {
          if (alarm_id != 0)
          {
            if (Alarms[alarm_id - 1].current_set == 255)
              Alarms[alarm_id - 1].current_set = 1;
            else if (Alarms[alarm_id - 1].current_set >= Alarms[alarm_id - 1].limit_max)
              Alarms[alarm_id - 1].current_set = 255;
            else
              Alarms[alarm_id - 1].current_set++;

          }
        }
        break;
      case 112 : if ( (current_screen  != MAX_SCREEN) || (alarm_id == 0) )
        {
          current_screen--;
          if (current_screen < 0) current_screen = MAX_SCREEN;
        }
        else
        {
          if (alarm_id != 0)
          {
            if (Alarms[alarm_id - 1].current_set <= 0)
              Alarms[alarm_id - 1].current_set = 255;
            else
              Alarms[alarm_id - 1].current_set--;
          }
        }
        break;
      case 208 : {
          if (current_screen  == MAX_SCREEN)
          {
            current_config++;
            if (current_config > MAX_CONFIG) current_config = 0;
          }
          break;
        }
      case 176 : {
          if (current_screen  == MAX_SCREEN)
          { current_config--; if (current_config < 0) current_config = MAX_CONFIG;
          }
          break;
        }
    }
  }
}
