/*
 JETI_EX_SENSOR.h, Version 0.1
 Mars 2012, by Denis Artru
 alarm and jetibox display are based on work of Uwe Gartmann and Alexander Buschek
 written for Arduino MEGA using rx2/tx2
 
 JETI_EX_SENSOR.h, Version 0.2
 2014, by DevFor8.com, info@devfor8.com
 modified for more items (15max)
 modified for external sending (software serial can be used)
 various fixes 
*/

#ifndef JETI_EX_SENSOR_h
#define JETI_EX_SENSOR_h
#include <inttypes.h>
#include <Arduino.h>

#define JETI_SENSOR_ID1 0x11 
#define JETI_SENSOR_ID2 0xA4
#define JETI_SENSOR_ID3 0xAD
#define JETI_SENSOR_ID4 0x04

#define LCDMaxPos 32

#define JB_key_none     0b0000
#define JB_key_up       0b0010
#define JB_key_right    0b0001
#define JB_key_down     0b0100
#define JB_key_left     0b1000

class JETI_Box_class {
private:
	int nbValueEX;
	const __FlashStringHelper * nameEX[16]; //zmenit na odkazy do PROGMEM
	const __FlashStringHelper * unitEX[16]; //zmenit na odkazy do PROGMEM
	uint8_t precisionEX[16]; /* 0 int; 1,2,3 float; 4 date, 5 time, 6 GPS*/ //zmenit na odkazy do PROGMEM
	int* valueEX[16];
	char alarmEX;
	uint8_t valueEXToSend;
public:
	uint8_t frame[180];
	int middle_bit9; //todle zmenime - false je pro prvni a posledni byte a pak pro byte uprostred mezi Ex datama a textama. Tj. staci nam znat offset tohodle a usetrime 64 bytes
	uint8_t frameSize;
   JETI_Box_class();
   void Init(const __FlashStringHelper* sensorName);
	uint8_t addData(const __FlashStringHelper* name, const __FlashStringHelper* unit);
	void setValue(uint8_t ident, short* valuePtr);
	void setValue(uint8_t ident, volatile float* valuePtr, uint8_t precision);
	void setValueTime(uint8_t ident, uint8_t[3]);
	void setValueDate(uint8_t ident, uint8_t[3]);
	void setValueGPS(uint8_t ident, uint8_t[4]); /* [0] NESW , [1] angle, [2][3] minute/1000*/
	void unsetValue(uint8_t ident);
    void JetiBox(const __FlashStringHelper* line1, const __FlashStringHelper* line2);
    void JetiBox(const char* line1, const char* line2);
    void JetiBox(const char* line);
	bool createFrame(uint8_t sendheader);
	
	void alarm(char alarmLetter);
   uint8_t readbuttons();
};

//extern JETI_Box_class JB;
extern uint8_t sensorFrameName;

#endif
