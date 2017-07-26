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

#include "JETI_EX_SENSOR.h"
#include "Arduino.h"
#include "SoftwareSerialO2.h"


SoftwareSerial JetiSerial(-1, -1);


unsigned char JETI_Box_class::WriteByte(unsigned char data, bool set9)
{
  JetiSerial.set9bit = set9;
  JetiSerial.write(data);
  JetiSerial.set9bit = 0;

  return data;
}

void EmptyISR()
{}

JETI_Box_class::JETI_Box_class() {
  // Class constructor
  alarmEX = 0;
  nbValueEX = 0;
  valueEXToSend = 0;
  FrameISR = EmptyISR;
}

void JETI_Box_class::Init(const __FlashStringHelper* sensorName, unsigned char port, short uartspeed) {
  nameEX[0] = sensorName;
  unitEX[0] = 0;
  valueEX[0] = 0;
  ItemInfo[0].Type = 0;
  ItemInfo[0].Precision  = 0;
  ItemInfo[0].NameLen  = strlen_P((PGM_P)sensorName);
  ItemInfo[0].TypeLen  = 0;

  nbValueEX = 1;

  JetiSerial.Init(port, port);
  JetiSerial.begin(uartspeed, true, 2, 1);
  JetiSerial.txMode();
}

unsigned char JETI_Box_class::addData(const __FlashStringHelper* name, const __FlashStringHelper* unit) {
  if (nbValueEX >= JETI_EX_MAXVALUES)
  {
    return 255;
  };
  // Prepare EX data
  nameEX[nbValueEX] = name;
  unitEX[nbValueEX] = unit;
  valueEX[nbValueEX] = 0;
  ItemInfo[nbValueEX].Type = 0;
  ItemInfo[nbValueEX].Precision  = 0;
  ItemInfo[nbValueEX].NameLen  = strlen_P((PGM_P)name);


  ItemInfo[nbValueEX].TypeLen  = strlen_P((PGM_P)unit);

  nbValueEX++;
  return (nbValueEX - 1);
}

void JETI_Box_class::setValue30(unsigned char ident, volatile int32_t* valuePtr, unsigned char precision) {
  if (ident >= JETI_EX_MAXVALUES)
    return;
  valueEX[ident] = (long*)valuePtr;
  ItemInfo[ident].Type = 8;
  ItemInfo[ident].Precision  = precision;
}

#if JETI_14

void JETI_Box_class::setValue14(unsigned char ident, volatile int16_t* valuePtr, unsigned char precision) {
  if (ident >= JETI_EX_MAXVALUES)
    return;

  valueEX[ident] = (long*)valuePtr;
  ItemInfo[ident].Type = 1;
  ItemInfo[ident].Precision  = precision;
}
#endif

#if JETI_6
void JETI_Box_class::setValue6(unsigned char ident, volatile int8_t* valuePtr, unsigned char precision) {
  if (ident >= JETI_EX_MAXVALUES)
  {
    return ;
  }
  valueEX[ident] = (long*)valuePtr;

  ItemInfo[ident].Type = 0;
  ItemInfo[ident].Precision  = precision;
}
#endif

#if JETI_DateTime

void JETI_Box_class::setValueDateTime(unsigned char ident, volatile DateTime* valuePtr, bool Date) //Date = true, Time = false
{
  if (ident >= JETI_EX_MAXVALUES)
    return;

  valueEX[ident] = (long*)valuePtr;
  ItemInfo[ident].Type = 5;
  ItemInfo[ident].Precision  = (byte)Date;
}
#endif

#if JETI_GPS
void JETI_Box_class::setValueGPS(unsigned char ident, volatile float* coordinate, bool Lo) {
  if (ident >= JETI_EX_MAXVALUES)
    return;

  valueEX[ident] = (long*)coordinate;
  ItemInfo[ident].Type = 9;
  ItemInfo[ident].Precision  = (byte)Lo;

}
#endif

void JETI_Box_class::unsetValue(unsigned char ident) {
  if (ident >= JETI_EX_MAXVALUES)
    return;

  valueEX[ident] = 0;
  ItemInfo[ident].Type = 0;
  ItemInfo[ident].Precision  = 0;
}

void JETI_Box_class::alarm(char alarmLetter) {
  alarmEX = alarmLetter;
}

void JETI_Box_class::JetiBox(const char* line1, const char* line2) {
  unsigned char i;
  unsigned char length;

  // 32 Byte Data Package
  length = strlen(line1);
  for (i = 0; i < (LCDMaxPos / 2); i++) {
    if (i < length) {
      jetiLcd[i] = line1[i];
    } else {
      jetiLcd[i] = ' ';
    }
  }

  length = strlen(line2);
  for (i = 0; i < (LCDMaxPos / 2); i++) {
    if (i < length) {
      jetiLcd[i + (LCDMaxPos / 2)] = line2[i];
    } else {
      jetiLcd[i + (LCDMaxPos / 2)] = ' ';
    }
  }
}

void JETI_Box_class::JetiBox(const char* line1)
{
  unsigned char i;
  unsigned char length;

  // 32 Byte Data Package
  length = strlen(line1);
  for (i = 0; i < (LCDMaxPos); i++) {
    if (i < length) {
      jetiLcd[i] = line1[i];
    } else {
      jetiLcd[i] = ' ';
    }
  }
}



void JETI_Box_class::JetiBox_P(const __FlashStringHelper* line1, const __FlashStringHelper* line2) {
  unsigned char i;
  unsigned char length;

  // 32 Byte Data Package
  length = strlen_P((PGM_P)line1);
  PGM_P p = reinterpret_cast<PGM_P>(line1);

  for (i = 0; i < (LCDMaxPos / 2); i++) {
    if (i < length) {
      unsigned char c = pgm_read_byte(p++);
      jetiLcd[i] = c;
    } else {
      jetiLcd[i] = ' ';
    }
  }

  p = reinterpret_cast<PGM_P>(line2);
  length = strlen_P((PGM_P)line2);
  for (i = 0; i < (LCDMaxPos / 2); i++) {
    if (i < length) {
      unsigned char c = pgm_read_byte(p++);
      jetiLcd[i + (LCDMaxPos / 2)] = c;
    } else {
      jetiLcd[i + (LCDMaxPos / 2)] = ' ';
    }
  }
}

/* 8-bit CRC polynomial  X^8 + X^2 + X + 1 */
#define POLY 0x07

unsigned char update_crc (unsigned char crc, unsigned char crc_seed)
{
  unsigned char crc_u;
  unsigned char i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i = 0; i < 8; i++)
  {
    crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
  }
  return crc_u;
}

unsigned char crc8fce (unsigned char *crc, unsigned char crc_lenght)
{
  unsigned char crc_up = 0;
  unsigned char c;
  for (c = 0; c < crc_lenght; c++)
  {
    crc_up = update_crc (crc[c], crc_up);
  }
  return crc_up;
}


unsigned char sensorFrameName = 0;

#define maxvals JETI_EX_MAXVALUES

unsigned char alarmCmpt = JETI_REPEAT_ALARM;
//const static unsigned char codage[4] = {0x52,0x1C,0x6C,0x23};

int JETI_Box_class::calculate_datalen(int valEXToSend, unsigned char* ValuesToSend)
{
  int cmpt = 6;
  int last_cmp = 6;
  (*ValuesToSend) = 0;

  for (int i_Loop = 0; i_Loop < maxvals; i_Loop++) {

    if (cmpt > 26)
    {
      cmpt = last_cmp;
      (*ValuesToSend)--;
      break;
    }

    int identSend = i_Loop + valEXToSend;
    if (identSend >= nbValueEX) {
      break;
    }
    if (identSend == 0) {
      continue;
    }

    last_cmp = cmpt;
    (*ValuesToSend)++;
    cmpt++; //identificator

    if (ItemInfo[identSend].Type == 8) {
      cmpt = cmpt + 4; //4 data bytes
    }
/*    else if (ItemInfo[identSend].Type == 4) {
      cmpt = cmpt + 3; //3 data bytes
    }*/
    #if JETI_14
    else if (ItemInfo[identSend].Type == 1) {
      cmpt = cmpt + 2; //two data bytes
    }
    #endif
    #if JETI_6
    else if (ItemInfo[identSend].Type == 0) {
      cmpt = cmpt + 1; //one data byte
    }
    #endif
    #if JETI_DateTime
    else if (ItemInfo[identSend].Type == 5) {
      cmpt = cmpt + 3; //4 data bytes
    }
    #endif
    #if JETI_GPS
    else if (ItemInfo[identSend].Type == 9)
    {
      cmpt = cmpt + 4;
    }
    #endif
  }

  return cmpt;
}

bool boxsent = false;

bool JETI_Box_class::SendFrame() {
  //unsigned char key;
  unsigned char i_Loop;
  unsigned char identSend = 0;
  unsigned char ValuesToSend = 0;
    #if JETI_6
    int8_t value8 = 0;
      #endif
    #if JETI_14
    int16_t value16 = 0;
      #endif
    #if JETI_GPS
    float valueGps = 0;
      #endif
    #if JETI_DateTime
    DateTime valueDT;
    #endif
  unsigned char b0;
  int32_t value32 = 0;
  bool sendheader = false;


  sendheader = (last_header + JETI_HEADERTIME) < millis() ;


  if (alarmEX) {
    WriteByte(0x7E, false);
    WriteByte(0x92, true);
    WriteByte(0x23, true);
    WriteByte(alarmEX, true);
    if (alarmCmpt) {
      alarmCmpt--;
    } else {
      alarmEX = 0;
      alarmCmpt = JETI_REPEAT_ALARM;
    }
  }
  else
  {
    WriteByte(0x7E, false);
    WriteByte(0x9F, true);

    unsigned char crc = 0;

    if (!sendheader)
    {
      while (valueEX[valueEXToSend] == 0) {
        valueEXToSend++;
        if (valueEXToSend >= nbValueEX) {
          valueEXToSend = 0;
          break;
        }
      }

      crc = update_crc(WriteByte(0x40 | calculate_datalen(valueEXToSend, &ValuesToSend) , true), crc);
    }
    else
    {
      unsigned char len = 8 + ItemInfo[sensorFrameName].NameLen + ItemInfo[sensorFrameName].TypeLen;
      crc = update_crc(WriteByte(len , true), crc);
    }


    crc = update_crc(WriteByte(JETI_SENSOR_ID1, true), crc);
    crc = update_crc(WriteByte(JETI_SENSOR_ID2, true), crc);
    crc = update_crc(WriteByte(Jeti_SensorID3, true), crc);
    crc = update_crc(WriteByte(Jeti_SensorID4, true), crc);

    crc = update_crc(WriteByte(0, true), crc);

    if (!sendheader) {

      uint8_t valsnd = valueEXToSend;

      for (i_Loop = 0; i_Loop < ValuesToSend; i_Loop++) { //two values only
        identSend = i_Loop + valsnd;
        valueEXToSend = identSend+1;

        if (identSend >= nbValueEX) {
          break;
        }
        if (identSend == 0) {
          continue;
        }

        //we can call it always as it is custom or empty
        FrameISR();


        unsigned char d = identSend << 4;

        switch (ItemInfo[identSend].Type) {
          case 8: //type 4 is not supported due memory optimization

            d |= 0x08;
            crc = update_crc(WriteByte(d, true), crc);

            value32 = *(int32_t*)valueEX[identSend];

            value32 = (value32 & 0x9FFFFFFF) | (ItemInfo[identSend].Precision << 29);

            crc = update_crc(WriteByte((unsigned char)(value32 & 0xFF), true), crc);
            crc = update_crc(WriteByte((unsigned char)((value32 >> 8) & 0xFF), true), crc);
            crc = update_crc(WriteByte((unsigned char)((value32 >> 16) & 0xFF), true), crc);

            value8 = (value32 >> 24) & 0xFF;
            value8 = (value8 & 0x9F) | (ItemInfo[identSend].Precision << 5);
            crc = update_crc(WriteByte((unsigned char)(value8), true), crc);

            break;
#if JETI_6
          case 0:
            value8 = *((int8_t*)valueEX[identSend]);
            d |= 0x00;
            crc = update_crc(WriteByte(d, true), crc);

            value8 = (value8 & 0x9F) | (ItemInfo[identSend].Precision << 5);
            crc = update_crc(WriteByte((unsigned char)(value8), true), crc);

            break;
#endif
#if JETI_14
          case 1:
            value16 = *((int16_t*)valueEX[identSend]);

            d |= 0x01;
            crc = update_crc(WriteByte(d, true), crc);
            value16 = (value16 & 0x9FFF) | (ItemInfo[identSend].Precision << 13);
            crc = update_crc(WriteByte((unsigned char)(value16 & 0xFF), true), crc);
            value8 = (value16 >> 8) & 0xFF;
            value8 = (value8 & 0x9F) | (ItemInfo[identSend].Precision << 5);
            crc = update_crc(WriteByte((unsigned char)(value8), true), crc);

            break;
#endif
#if JETI_DateTime            
          case 5: //Time

            d |= 0x05;
            crc = update_crc(WriteByte(d, true), crc);

            valueDT = *(DateTime*)valueEX[identSend];
        
            b0 = valueDT.DayHrs;
            b0 |= (ItemInfo[identSend].Precision == 1) ? 0x40 : 0; //date/time

            crc = update_crc(WriteByte((unsigned char)(valueDT.YearSec), true), crc);
            crc = update_crc(WriteByte((unsigned char)(valueDT.MonMin), true), crc);
            crc = update_crc(WriteByte((unsigned char)(b0), true), crc);


            break;
#endif
#if JETI_GPS
          case 9: //GPS

            d |= 0x09;
            crc = update_crc(WriteByte(d, true), crc);

            valueGps = *(float*)valueEX[identSend];
            uint16_t deg = abs(valueGps);
            float fmin = (valueGps - deg) * 0.6f * 100000;
            uint16_t mi = abs(fmin);

            b0 = (deg >> 8 ) & 0x01;
            b0 |= (ItemInfo[identSend].Precision == 1) ? 0x20 : 0; //lat/long
            b0 |= (valueGps < 0) ? 0x40 : 0; //direction

            crc = update_crc(WriteByte((unsigned char)(mi & 0xFF), true), crc);
            crc = update_crc(WriteByte((unsigned char)((mi >> 8) & 0xFF), true), crc);
            crc = update_crc(WriteByte((unsigned char)(deg & 0xFF), true), crc);
            crc = update_crc(WriteByte((unsigned char)(b0), true), crc);


            break;
#endif
        }
      }

      if (valueEXToSend >= nbValueEX - 1) {
        valueEXToSend = 0;
      }

    } else {
      crc = update_crc(WriteByte(sensorFrameName, true), crc);

      int d = (ItemInfo[sensorFrameName].NameLen << 3) | ItemInfo[sensorFrameName].TypeLen;

      crc = update_crc(WriteByte(d, true), crc);

      PGM_P p = reinterpret_cast<PGM_P>(nameEX[sensorFrameName]);
      for (i_Loop = 0; i_Loop < ItemInfo[sensorFrameName].NameLen; i_Loop++)
      {
        unsigned char c = pgm_read_byte(p++);
        crc = update_crc(WriteByte(c, true), crc);
      }
      p = reinterpret_cast<PGM_P>(unitEX[sensorFrameName]);
      for (i_Loop = 0; i_Loop < ItemInfo[sensorFrameName].TypeLen; i_Loop++)
      {
        unsigned char c = pgm_read_byte(p++);
        crc = update_crc(WriteByte(c, true), crc);
      }
      sensorFrameName++;
      if (sensorFrameName >= nbValueEX)
      {
        sensorFrameName = 0;
      }
      last_header = millis();

    }
    // Finish the EX frame; //
    WriteByte(crc, true);
  }

  //we can call it always as it is custom or empty
  FrameISR();

  // Send the value to print on jetiBox 2*16 characters
  WriteByte(0xFE, false);
  for (i_Loop = 0; i_Loop < LCDMaxPos; i_Loop++) {
    WriteByte(jetiLcd[i_Loop], true);
  }
    WriteByte(0xFF, false);

  
  return true;
}

unsigned char JETI_Box_class::readbuttons()
{
  unsigned char buttons = -1;

  if (JetiSerial.available() > 0 )
  {
    buttons = JetiSerial.read();

    if ( (lastbuttons != buttons) || (lastkeytime < millis() ) )
    {
      lastkeytime = millis() + JETI_LASTKEYTIME;
      return lastbuttons = buttons;
    } else
    {
      return 0;
    }
  }
  else
    return 0;

}

void JETI_Box_class::rxMode()
{
  JetiSerial.rxMode();
  JetiSerial.listen();
}

void JETI_Box_class::txMode()
{
  JetiSerial.stopListening();
  JetiSerial.txMode();
}


