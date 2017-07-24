/*
  JETI_EX_SENSOR.cpp, Version 0.1
  Mars 2012, by Denis Artru
  alarm and jetibox display are based on work of Uwe Gartmann and Alexander Buschek
  written for Arduino MEGA using rx2/tx2

   JETI_EX_SENSOR.h, Version 0.2
   2014, by DevFor8.com, info@devfor8.com
   modified for more items (15max)
   modified for external sending (software serial can be used)
   various fixes

   JETI_EX_SENSOR.h, Version 0.3
   2017, by Radek Voltr, voltr@voltr.eu
   fixes
   data types extensions
   memory optimizations
*/

#ifndef JETI_EX_SENSOR_h
#define JETI_EX_SENSOR_h
#include "SoftwareSerialO2.h"
#include "jeticfg.h"

#define JETI_SENSOR_ID1 0x11
#define JETI_SENSOR_ID2 0xA4

#define LCDMaxPos 32


extern const unsigned char Jeti_SensorID3;
extern const unsigned char Jeti_SensorID4;
extern SoftwareSerial JetiSerial;


#define JB_key_none     0b0000
#define JB_key_up       0b0010
#define JB_key_right    0b0001
#define JB_key_down     0b0100
#define JB_key_left     0b1000

#if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
typedef fstr_t __FlashStringHelper;
#endif

struct ItemType {
  unsigned short Type  : 5;    // 0..7   (3 bits)
  unsigned short Precision : 3;    // 0..3  (2 bits)
  unsigned short NameLen  : 5;
  unsigned short TypeLen : 3;
};

#if JETI_DateTime
struct DateTime
{
  uint8_t DayHrs;
  uint8_t MonMin;
  uint8_t YearSec;
};
#endif

class JETI_Box_class {
  private:
    int nbValueEX;
    const __FlashStringHelper* nameEX[JETI_EX_MAXVALUES];
    const __FlashStringHelper* unitEX[JETI_EX_MAXVALUES];
    ItemType ItemInfo[JETI_EX_MAXVALUES]; /* according to JETI documentation*/
    volatile long* valueEX[JETI_EX_MAXVALUES];
    char alarmEX;
    unsigned char valueEXToSend;
    long lastkeytime = 0;
    unsigned char jetiLcd[LCDMaxPos];

    unsigned char buttons;
    unsigned char lastbuttons;

    long last_header;

    int calculate_datalen(int valEXToSend, unsigned char* ValuesToSend);

  public:
    JETI_Box_class();
    void Init(const __FlashStringHelper* sensorName, unsigned char port, short uartspeed);
    unsigned char addData(const __FlashStringHelper* name, const __FlashStringHelper* unit);
    void setValue30(unsigned char ident, volatile int32_t* valuePtr, unsigned char precision);
#if JETI_14
    void setValue14(unsigned char ident, volatile int16_t* valuePtr, unsigned char precision);
#endif
#if JETI_6
    void setValue6(unsigned char ident, volatile int8_t* valuePtr, unsigned char precision);
#endif
#if JETI_DateTime
    void setValueDateTime(unsigned char ident, volatile DateTime* valuePtr, bool Date); //Date = true, Time = false
#endif
#if JETI_GPS
    void setValueGPS(unsigned char ident, volatile float* coordinate, bool Lo); //Lo = true, La = false
#endif
    void unsetValue(unsigned char ident);
    void JetiBox(const char* line1, const char* line2);
    void JetiBox(const char* line1);
    void JetiBox_P(const __FlashStringHelper* line1, const __FlashStringHelper* line2);
    bool SendFrame();

    void alarm(char alarmLetter);
    unsigned char readbuttons();
    void rxMode();
    void txMode();
};

#endif
