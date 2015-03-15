#include <EEPROM.h>

/*
	 Mav2DupEx Version 1.0
	 2014, by DevFor8.com, info@devfor8.com
	 
	 part of code is based on ArduCAM OSD
         Bugfixes and some enhancements added by MTBSTEVE
*/

#include <SoftwareSerialO2.h>
#include <FastSerial.h>
#include <GCS_MAVLink.h>
#include <JETI_EX_SENSOR.h>
#include <c++.h>
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

#define TELEMETRY_SPEED  57600  // How fast our MAVLink telemetry is coming to Serial port, 57600 for sure, 115200 better

//config data
//zkusme to seknout do progmem
#define ITEMNAME_1 F("Armed")
#define ITEMTYPE_1 F("")
#define ITEMVAL_1 (short*)&motor_armed

#define ITEMNAME_2 F("Batt %")
#define ITEMTYPE_2 F("%")
#define ITEMVAL_2 (short*)&osd_battery_remaining_A

#define ITEMNAME_3 F("Batt V")
#define ITEMTYPE_3 F("V")
#define ITEMVAL_3 &osd_vbat_A

#define ITEMNAME_4 F("Batt A")
#define ITEMTYPE_4 F("A")
#define ITEMVAL_4  &osd_curr_A

#define ITEMNAME_5 F("Batt C")
#define ITEMTYPE_5 F("mAh")
#define ITEMVAL_5  &osd_capacity_mA

/* the following parameters are not used by me
#define ITEMNAME_6 F("Yaw")
#define ITEMTYPE_6 F("Deg")
#define ITEMVAL_6 (short*)&osd_yaw

#define ITEMNAME_7 F("Pitch")
#define ITEMTYPE_7 F("Deg")
#define ITEMVAL_7 (short*)&osd_pitch
//#define ITEMVAL_7 (short*)&debug1

#define ITEMNAME_8 F("Roll")
#define ITEMTYPE_8 F("Deg")
#define ITEMVAL_8 (short*)&osd_roll
//#define ITEMVAL_8 (short*)&debug2
*/
// Climb rate and GPS pos added
#define ITEMNAME_6 "Climb"
#define ITEMTYPE_6 "m/s"
#define ITEMVAL_6  &osd_climb

#define ITEMNAME_7 "Lat"
#define ITEMTYPE_7 ""
#define ITEMVAL_7 (float*)&gps_lat

#define ITEMNAME_8 "Lon"
#define ITEMTYPE_8 ""
#define ITEMVAL_8 (float*)&gps_lon

#define ITEMNAME_9 F("Alt")
#define ITEMTYPE_9 F("m")
#define ITEMVAL_9 &osd_alt
//#define ITEMVAL_9 &debug3

#define ITEMNAME_10 F("Home Dir")
#define ITEMTYPE_10 F("Deg")
#define ITEMVAL_10 (short*)&osd_home_heading

#define ITEMNAME_11 F("Home Dist")
#define ITEMTYPE_11 F("m")
#define ITEMVAL_11 (short*)&osd_home_distance

#define ITEMNAME_12 "HDOP"
#define ITEMTYPE_12 ""
#define ITEMVAL_12 &ap_gps_hdop

#define ITEMNAME_13 "GPS Lock"
#define ITEMTYPE_13 ""
#define ITEMVAL_13 (short*)&osd_fix_type_jeti

#define ITEMNAME_14 "GPS Sat"
#define ITEMTYPE_14 ""
#define ITEMVAL_14 (short*)&osd_satellites_visible

#define ITEMNAME_15 "Speed"
#define ITEMTYPE_15 "m/s"
#define ITEMVAL_15  &osd_groundspeed



#define ABOUT_1 F("MavToDuplex 1.0")
#define ABOUT_2 F("Nav with < >")



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
	{read = JetiSerial.read();
          }
  
  long wait = (millis()-time) - GETCHAR_TIMEOUT_ms;

  if (wait > 0)
  delay(wait);
  
  return read;
}

//char * floatToString(char * outstr, float value, int places, int minwidth=, bool rightjustify) {
char * floatToString(char * outstr, float value, int places, int minwidth=0) {
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
        d/= 10.0;    
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
    if (minwidth > charcount){        
        extra = minwidth - charcount;
        charcount = minwidth;
    }


    // write out the negative if needed
    if (value < 0)
        outstr[c++] = '-';

    if (tenscount == 0) 
        outstr[c++] = '0';

    for (i=0; i< tenscount; i++) {
        digit = (int) (tempfloat/tens);
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
        for (int i = 0; i< extra; i++) {
            outstr[c++] = ' ';
        }
    }


    outstr[c++] = '\0';
    return outstr;
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
    boolean bit9 = false;
  
    for (int i = 0 ; i<JB.frameSize ; i++ )
    {
      if (i == 0)
        bit9 = false;
      else
      if (i == JB.frameSize-1)
       bit9 = false;
      else
      if (i == JB.middle_bit9)
       bit9 = false;
      else
       bit9 = true;

      
      JetiTransmitByte(JB.frame[i], bit9);
      
      //can we do check between frames ?
      if (Serial.available() > 0) //at least beat should fill
        read_mavlink(0);      //only one
      
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
  
  
  Alarms[1].name =F( "Max Capacity");
  Alarms[1].type =F( "mAh");
  Alarms[1].limit_max = 200;
  Alarms[1].limit_min = 1;
  Alarms[1].val_step = 100;
  Alarms[1].condition = COND_MORE_EQUAL;
  Alarms[1].current_set = EEPROM.read(1);
  Alarms[1].alarm = 'C';
  Alarms[1].reset_alarm = 0;
  
  Alarms[2].name =F( "Max Altitude");
  Alarms[2].type =F( "m");
  Alarms[2].limit_max = 200;
  Alarms[2].limit_min = 1;
  Alarms[2].val_step = 5;
  Alarms[2].condition = COND_MORE_EQUAL;
  Alarms[2].current_set = EEPROM.read(2);
  Alarms[2].alarm = 'A';
  Alarms[2].reset_alarm = 0;
  
  Alarms[3].name =F( "Max Distance");
  Alarms[3].type =F( "m");
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
      else
      if ((Alarms[id].condition == COND_LES_EQUAL) && (Alarms[id].current_set != 255) )      {
        if ( (current_val > 0) && (current_val <= Alarms[id].current_set * Alarms[id].val_step) )
          {
             //Serial.print("Alarm Les:");Serial.print(id);Serial.print(" ");Serial.print(current_val);Serial.print(" ");Serial.print(Alarms[id].current_set * Alarms[id].val_step);Serial.print(" ");Serial.println(Alarms[id].current_set);
             JB.alarm(Alarms[id].alarm);
          }
      }
}

void setup() 
{
    pinMode(13, OUTPUT);     
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    Serial.begin(TELEMETRY_SPEED,320,0);
    Serial.println(F("Ready")); //progmem
  
    pinMode(JETI_RX, OUTPUT);
    pinMode(3,OUTPUT);
    digitalWrite(3,LOW);
    
    strcpy_P((char*)&LastMessage,(prog_char*)F("Mav2Dup Init OK"));
    
    InitAlarms();
    
    displayMemory();
    
    // setup mavlink port
    mavlink_comm_0_port = &Serial;
    JetiUartInit();          // Requires JETIBOX.INO library...
    Serial.println(F("JetiUartInit")); //progmem

    JB.JetiBox(ABOUT_1,ABOUT_2); //change to this for copy directly from F() and also to AddData in F() form only
    JB.Init(F("Mav2Dup"));
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
//    JB.addData(ITEMNAME_15,ITEMTYPE_15);
    
    //values are linked so we don't need setValue during every change but one time is enoug
    
    JB.setValue(1,ITEMVAL_1);
    JB.setValue(2,ITEMVAL_2);
    JB.setValue(3,ITEMVAL_3,1);
    JB.setValue(4,ITEMVAL_4,1);
    JB.setValue(5,ITEMVAL_5);
    JB.setValue(6,ITEMVAL_6,1);
    JB.setValue(7,ITEMVAL_7,6);
    JB.setValue(8,ITEMVAL_8,6);
    JB.setValue(9,ITEMVAL_9,1);
    JB.setValue(10,ITEMVAL_10);
    JB.setValue(11,ITEMVAL_11);
    JB.setValue(12,ITEMVAL_12,2);
    JB.setValue(13,ITEMVAL_13);
    JB.setValue(14,ITEMVAL_14);
    JB.setValue(15,ITEMVAL_15,1);
    
    do {
      JB.createFrame(1);
      SendFrame();
      delay(GETCHAR_TIMEOUT_ms);
      //Serial.write(sensorFrameName);
    }
    while (sensorFrameName != 0);
    //Serial.print("Ready done");

    digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
       
    displayMemory();
 }




int header = 0;
int lastbtn = 240;
int current_screen = 1; // 0 - about , 2 - message ,1 - prestart (arm/gps mode/gps count/ voltage/ one line message), 3- flight (mode, home angle, home alt) 4 - power state (v/cur/capacity/perc), 5 IMU status (angles, alt), 6 - Alarms
int current_config = 0; // 0 - Alarms, 1 - capacity warn, 2 - dist warn

int alarm_id = -1;
float alarm_current = 0;


char temp[LCDMaxPos/2];
char msg_line1[LCDMaxPos/2];
char msg_line2[LCDMaxPos/2];

//int8_t last_mode = -1; 

void process_screens()
{
  


  switch (current_screen)
  {
    case 0 :{ JB.JetiBox(ABOUT_1,ABOUT_2);   break;}
    case 2 :{ JB.JetiBox(LastMessage);   break;}    
    case 1 :{ 
      msg_line1[0] = 0;      msg_line2[0] = 0;
      if (motor_armed == 1)
        strcat_P((char*)&msg_line1,(prog_char*)F("ARM "));
        else
        strcat_P((char*)&msg_line1,(prog_char*)F("DIS "));

      if (osd_fix_type_jeti == 0)
        strcat_P((char*)&msg_line1,(prog_char*)F("NO:"));
        else
        if (osd_fix_type_jeti == 2)
          strcat_P((char*)&msg_line1,(prog_char*)F("2D:"));
          else
          if (osd_fix_type_jeti == 2)
            strcat_P((char*)&msg_line1,(prog_char*)F("3D:"));        

      temp[0] = 0;
      int i = osd_satellites_visible;
      if (i<0)
        i = 0;
        else
        if (i>99)
          i = 99;
      itoa (i,(char*)&temp,10);
      strcat((char*)&msg_line1,(char*)&temp);
      strcat_P((char*)&msg_line1,(prog_char*)F(" "));
      
      temp[0] = 0;
      floatToString((char*)&temp,osd_vbat_A,1);
      strcat((char*)&msg_line1,(char*)&temp);
      strcat_P((char*)&msg_line1,(prog_char*)F("V"));

      strncpy((char*)&msg_line2,(char*)LastMessage,LCDMaxPos/2);
      
      
      JB.JetiBox((char*)&msg_line1,(char*)&msg_line2);
    break;}    
    case 3 :{ 
    
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
    
 
      msg_line1[0] = 0;      msg_line2[0] = 0;
        strcat_P((char*)&msg_line1,(prog_char*)mode_str);
       strcat_P((char*)&msg_line1,(prog_char*)F(" Alt:"));
      temp[0] = 0;
      floatToString((char*)&temp,osd_alt,1);
      strcat((char*)&msg_line1,(char*)&temp);
      strcat_P((char*)&msg_line1,(prog_char*)F("m"));
      
      strcat_P((char*)&msg_line2,(prog_char*)F("Dis:"));
     temp[0] = 0;
      //floatToString((char*)&temp,osd_home_distance,0);
      itoa(osd_home_distance,(char*)&temp,10);
      strcat((char*)&msg_line2,(char*)&temp);
      strcat_P((char*)&msg_line2,(prog_char*)F("m Dir:"));
      itoa (osd_home_heading,(char*)&temp,10);
      strcat((char*)&msg_line2,(char*)&temp);
      strcat_P((char*)&msg_line2,(prog_char*)F("°"));
      

      JB.JetiBox((char*)&msg_line1,(char*)&msg_line2);
      break;}    
    case 4 :{    
      msg_line1[0] = 0;      msg_line2[0] = 0;
      temp[0] = 0;
      strcat_P((char*)&msg_line1,(prog_char*)F("Power "));

      floatToString((char*)&temp,osd_vbat_A,1);
      strcat((char*)&msg_line1,(char*)&temp);
      strcat_P((char*)&msg_line1,(prog_char*)F("V "));
      
      temp[0] = 0;
      itoa(osd_curr_A,(char*)&temp,10);
      strcat((char*)&msg_line2,(char*)&temp);
      strcat_P((char*)&msg_line2,(prog_char*)F("A "));

      itoa (osd_capacity_mA,(char*)&temp,10);
      strcat((char*)&msg_line2,(char*)&temp);
      strcat_P((char*)&msg_line2,(prog_char*)F("mAh"));      
      
      JB.JetiBox((char*)&msg_line1,(char*)&msg_line2);
      break;}    
    case 5 :{    
      msg_line1[0] = 0;      msg_line2[0] = 0;
      strcat_P((char*)&msg_line1,(prog_char*)F("Rol:"));
      temp[0] = 0;
      itoa(osd_roll,(char*)&temp,10);
      strcat((char*)&msg_line1,(char*)&temp);
      strcat_P((char*)&msg_line1,(prog_char*)F(" Yaw:"));
      itoa (osd_yaw,(char*)&temp,10);
      strcat((char*)&msg_line1,(char*)&temp);

      strcat_P((char*)&msg_line2,(prog_char*)F("Pit:"));
      temp[0] = 0;
      itoa(osd_pitch,(char*)&temp,10);
      strcat((char*)&msg_line2,(char*)&temp);
      strcat_P((char*)&msg_line2,(prog_char*)F(" Alt:"));
      itoa (osd_alt,(char*)&temp,10);
      strcat((char*)&msg_line2,(char*)&temp);

      JB.JetiBox((char*)&msg_line1,(char*)&msg_line2);
      break;}    
    case MAX_SCREEN :{ 

      msg_line1[0] = 0;      msg_line2[0] = 0;
      
        if (alarm_id != current_config)
          {
            if (alarm_id !=0)
              {
                //store to eprom
                EEPROM.write(alarm_id-1,Alarms[alarm_id-1].current_set);
              }
            if (current_config !=0)
              {
               //read from array to  
                
                
              }
            alarm_id = current_config;
          }
      
          
      if (current_config == 0)
        {strcpy_P((char*)&msg_line1,(prog_char*)F("Alarms"));strcpy_P((char*)&msg_line2,(prog_char*)F("Use Up/Down"));}
        else
        {
          // Serial.print(Alarms[alarm_id-1].name); Serial.print(" ");
          strcpy_P((char*)&msg_line1,(prog_char*)Alarms[alarm_id-1].name);
          //Serial.println(msg_line1);
          
          if (Alarms[alarm_id-1].current_set == 255)         
            strcpy_P((char*)&msg_line2,(prog_char*)F("<> Not Active"));
            else
            {
            if (Alarms[alarm_id-1].condition == COND_MORE_EQUAL)
                strcpy_P((char*)&msg_line2,(prog_char*)F(" > "));
              else
                if (Alarms[alarm_id-1].condition == COND_LES_EQUAL)
                    strcpy_P((char*)&msg_line2,(prog_char*)F(" < "));
                         
              floatToString((char*)&temp,Alarms[alarm_id-1].val_step * Alarms[alarm_id-1].current_set ,1);
              strcat((char*)&msg_line2,(char*)&temp);
              strcat_P((char*)&msg_line2,(prog_char*)F(" "));                            
              strcat_P((char*)&msg_line2,(prog_char*)Alarms[alarm_id-1].type);                            
            }
        }
      
      JB.JetiBox((char*)&msg_line1,(char*)&msg_line2);  
    }    
  }
   
    //process alarms
  ProcessAlarm(0,osd_vbat_A);
  ProcessAlarm(1,osd_capacity_mA);
  ProcessAlarm(2,osd_alt);
  ProcessAlarm(3,osd_home_distance);
  
  
  
}

void loop() 
{
    unsigned long time = millis();

    SendFrame();
    
    //Serial.print("Send frame :");Serial.println(millis()-time);
    time = millis();

  int read = 0;
 
  pinMode(JETI_RX, INPUT);
  pinMode(JETI_TX, INPUT_PULLUP);

//  digitalWrite(JETI_TX,LOW);

  JetiSerial.listen();
  JetiSerial.flush();
  
  
  while ( JetiSerial.available()  == 0 )
  {
//    if (Serial.available() > 0) //at least beat should fill
//      read_mavlink(5);
    
    if (millis()-time >  5) //5ms to waiting
      break; // return, if timout occures
  }
  
  if (JetiSerial.available() >0 )
	{read = JetiSerial.read();
    //240 = no buttons
    //224 - right
    //112 - left
    //208 up
    //176 down
    //144 up+down
    //96 left+right
    if (lastbtn != read)
    {
        //Serial.println(read);
        lastbtn = read;
        // process buttons
        switch (read)
        {
          case 224 : if ( (current_screen  != MAX_SCREEN) || (alarm_id ==0) )
                        { current_screen++; if (current_screen >MAX_SCREEN) current_screen=0; }
                        else
                        { 
                        if (alarm_id !=0)
                        {
                          if (Alarms[alarm_id-1].current_set == 255)
                            Alarms[alarm_id-1].current_set = 1;
                          else
                            if (Alarms[alarm_id-1].current_set >= Alarms[alarm_id-1].limit_max)
                              Alarms[alarm_id-1].current_set = 255;
                              else
                              Alarms[alarm_id-1].current_set++;
                          
                        }
                        }
                        break;
          case 112 : if ( (current_screen  != MAX_SCREEN) || (alarm_id ==0) )
                      { current_screen--; if (current_screen <0) current_screen = MAX_SCREEN; }        
                      else
                      {
                        if (alarm_id !=0)
                        {
                          if (Alarms[alarm_id-1].current_set <= 0)
                            Alarms[alarm_id-1].current_set = 255;
                            else
                            Alarms[alarm_id-1].current_set--;
                        }
                      }
                      break;
          case 208 : {
              if (current_screen  == MAX_SCREEN) 
                { current_config++; if (current_config > MAX_CONFIG) current_config = 0;}  
          break;}        
          case 176 : {
              if (current_screen  == MAX_SCREEN) 
                { current_config--; if (current_config <0) current_config = MAX_CONFIG;
                }  
          break;}        
          }
    }
  }
    
//    if (Serial.available() > 0) //at least beat should fill
//      read_mavlink(5);
    
   if (current_screen !=MAX_SCREEN)
     current_config = 0; //zero 5th screen
     
	process_screens();
   
//    if (Serial.available() > 0) //at least beat should fill
//      read_mavlink(5);

    setHomeVars();   // calculate and set Distance from home and Direction to home
    calcGPS(); // recalc GPS vars for duplex fix by rosewhite
    
//    if (Serial.available() > 0) //at least beat should fill
//      read_mavlink(5);

    // prepare frame
    header++;
    if (header >= 5)
    {
      JB.createFrame(1);
      header = 0;

    }
    else
    {
      JB.createFrame(0);
    }
    
    //Serial.print("frameSize:");Serial.println(JB.frameSize);
    // wait rest of time to 20ms
    //read_mavlink(); //read again to not skip dat   

   pinMode(JETI_RX, OUTPUT);
  
  
    if (Serial.available() > 0) //at least beat should fill
      read_mavlink(10);

   long wait = GETCHAR_TIMEOUT_ms;
   long milli = millis()-time;

   if (milli > wait)
     wait = 0;
     else
     wait = wait - milli;

  
  while ( millis()-time <  20 )
  {
    if (Serial.available() > 0) //at least beat should fill
      read_mavlink(10);
      else
      delay(1);
  }
  
  //Serial.print("Real wait :");Serial.println(millis()-time);

//    Serial.print("Wait:");Serial.println(wait);
//    if (wait > 0)
//      delay(wait);
      
  pinMode(JETI_TX, OUTPUT);
  
  
  //debug2 = wait;

}
