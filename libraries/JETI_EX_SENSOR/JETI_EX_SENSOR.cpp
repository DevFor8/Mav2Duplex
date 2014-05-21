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
*/
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <JETI_EX_SENSOR.h>

#define LCDBufferSize LCDMaxPos + 1
#define JETI_WAIT 3
uint8_t jetiLcd[LCDMaxPos];
uint8_t buttons;
uint8_t lastbuttons;


const byte crctable[256] PROGMEM = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
	0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
	0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
	0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
	0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
	0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
	0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
	0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
	0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
	0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};


JETI_Box_class::JETI_Box_class() {
// Class constructor
	alarmEX=0;
	nbValueEX=0;
	frameSize=0;
	valueEXToSend=0;
}

void JETI_Box_class::Init(const char* sensorName) {
	nameEX[0]=(char*)sensorName;
	unitEX[0]=" ";
	nbValueEX++;
}

uint8_t JETI_Box_class::addData(const char* name, char* unit) {
   // Prepare EX data
   nameEX[nbValueEX]=(char*)name;
	unitEX[nbValueEX]=(char*)unit;
	valueEX[nbValueEX]=0;
	precisionEX[nbValueEX]=0;
	nbValueEX++;
	return(nbValueEX-1);
}

void JETI_Box_class::setValue(uint8_t ident, short* valuePtr) {
	valueEX[ident]=(int*)valuePtr;
	precisionEX[ident]=0;
}

void JETI_Box_class::setValue(uint8_t ident, float* valuePtr, uint8_t precision) {
	valueEX[ident]=(int*)valuePtr;
	precisionEX[ident]=precision;
}
void JETI_Box_class::setValueDate(uint8_t ident, uint8_t date[3]) {
	valueEX[ident]=(int*)date;
	precisionEX[ident]=4;
}
void JETI_Box_class::setValueTime(uint8_t ident, uint8_t time[3]) {
	valueEX[ident]=(int*)time;
	precisionEX[ident]=5;
}
void JETI_Box_class::setValueGPS(uint8_t ident, uint8_t coordinate[4]) {
	valueEX[ident]=(int*)coordinate;
	precisionEX[ident]=6;
}
void JETI_Box_class::unsetValue(uint8_t ident) {
	valueEX[ident]=0;
	precisionEX[ident]=0;
}

void JETI_Box_class::alarm(char alarmLetter) {
	alarmEX=alarmLetter;
}

void JETI_Box_class::JetiBox(const char* line1, const char* line2) {
  uint8_t i;
  uint8_t length;

  // 34 Byte Data Package
  length = strlen(line1);
  for (i=0; i<(LCDMaxPos/2);i++) {
    if (i<length) {
      jetiLcd[i] = line1[i];
    } else {
      jetiLcd[i] = ' ';
    }
  }
  length = strlen(line2);
  for (i=0; i<(LCDMaxPos/2);i++) {
    if (i<length) {
      jetiLcd[i+(LCDMaxPos/2)] = line2[i];
    } else {
      jetiLcd[i+(LCDMaxPos/2)] = ' ';
    }
  } 
}

/* 8-bit CRC polynomial X^8 + X^2 + X + 1 */
#define POLY 0x07
uint8_t update_crc (uint8_t crc, uint8_t crc_seed)
{
uint8_t crc_u;
uint8_t i;
crc_u = crc;
crc_u ^= crc_seed;
for (i=0; i<8; i++)
{
crc_u = ( crc_u & 0x80 ) ? POLY ^ ( crc_u << 1 ) : ( crc_u << 1 );
}
return crc_u;
}
uint8_t crc8fce (uint8_t *crc, uint8_t crc_lenght)
{
uint8_t crc_up = 0;
uint8_t c;
for(c=0;c < crc_lenght; c++) {
crc_up = update_crc (crc[c], crc_up);
}
return crc_up;
}

uint16_t uint14 (long value)                            
{
  if (value < 0)
    return ((value >> 8) & 0x1F) | 0x80;       	
  else
    return (value >> 8);                   	
}

uint8_t sensorFrameName = 0;

#define maxvals 5

#define JETI_REPEAT_ALARM 6
uint8_t alarmCmpt = JETI_REPEAT_ALARM;
const static uint8_t codage[4] = {0x52,0x1C,0x6C,0x23};
bool JETI_Box_class::createFrame(uint8_t sendheader) {
	//uint8_t key;
	uint8_t crc8=0;
	uint8_t i_Loop;
	uint8_t identSend;
	short value;
	uint8_t *valeur;
	int cmpt=0;
	if (alarmEX) {
	   frame[cmpt]=0x7E;
		bit9[cmpt]=false;
		cmpt++;
		frame[cmpt]=0x92;
		bit9[cmpt]=true;
		cmpt++;
		frame[cmpt]=0x23;
		bit9[cmpt]=true;
		cmpt++;
		frame[cmpt]=alarmEX;
		bit9[cmpt]=true;
		cmpt++;
		if (alarmCmpt) {
			alarmCmpt--;
		} else {
			alarmEX=0;
			alarmCmpt=JETI_REPEAT_ALARM;
		}
	}
	if (!alarmEX ) {
	   frame[0]=0x7E;
		bit9[0]=false;
	   frame[1]=0x9F;
		bit9[1]=true;
	   frame[2]=0x00;
		bit9[2]=true;
	   frame[3]=JETI_SENSOR_ID1;
		bit9[3]=true;
	   frame[4]=JETI_SENSOR_ID2;
		bit9[4]=true;
	   frame[5]=JETI_SENSOR_ID3;
		bit9[5]=true;
	   frame[6]=JETI_SENSOR_ID4;
		bit9[6]=true;
		//key=(uint8_t)random(0xFE)+1;
		frame[7]=0;
		bit9[7]=true;
		if (sendheader == 0) {
			cmpt=8;
			frame[2]=0x40;
			while (valueEX[valueEXToSend]==0) {
				valueEXToSend++;
				if (valueEXToSend>=nbValueEX) {
					valueEXToSend=0;
					break;
				}
			}
			//valueEXToSend=1+((valueEXToSend-1)/3)*3;
			for (i_Loop=0; i_Loop<maxvals; i_Loop++) { //two values only
				identSend=i_Loop+valueEXToSend;
				if (identSend>=nbValueEX) {
								break;
				}
				if (valueEX[identSend]==0) {
					continue;
				}
				frame[cmpt]=0;
				// 5 for date/time 3 octets
				// 9 for GPS 4 octets
				frame[cmpt]=identSend<<4;
				bit9[cmpt]=true;
				switch (precisionEX[identSend]) {
					case 0:
						value=*((short*)valueEX[identSend]);
						break;
					case 1:
					case 2:
					case 3:
						value=round(*((float*)valueEX[identSend])*pow(10,precisionEX[identSend]));
						break;
				}
				if (precisionEX[identSend]<4) { 
						frame[cmpt]|=0x01;
						cmpt++;
						value=(value&0x9FFF)|(precisionEX[identSend]<<13);
						frame[cmpt]=(uint8_t)byte(value&0xFF);
						bit9[cmpt]=true;
						cmpt++;
						frame[cmpt]=(uint8_t)byte((value>>8)&0xFF);
						bit9[cmpt]=true;
						cmpt++;
				} else if (precisionEX[identSend]<6) {
					frame[cmpt]|=0x05;
					cmpt++;
					valeur=((uint8_t*)valueEX[identSend]);
					frame[cmpt]=valeur[2];
					bit9[cmpt]=true;
					cmpt++;
					frame[cmpt]=valeur[1];
					bit9[cmpt]=true;
					cmpt++;
					frame[cmpt]=valeur[0];
					frame[cmpt]&=0x1F;
					bit9[cmpt]=true;
					if (precisionEX[identSend]==4) {
						frame[cmpt]|=0x20;
					}
					cmpt++;
				} else if (precisionEX[identSend]==6) {
					frame[cmpt]|=0x09;
					cmpt++;
					valeur=((uint8_t*)valueEX[identSend]);
					frame[cmpt]=valeur[3];
					bit9[cmpt]=true;
					cmpt++;
					frame[cmpt]=valeur[2];
					bit9[cmpt]=true;
					cmpt++;
					frame[cmpt]=valeur[1];
					bit9[cmpt]=true;
					cmpt++;
					frame[cmpt]=valeur[0];
					bit9[cmpt]=true;
					switch (valeur[0]) {
						case 'N':
							frame[cmpt]=0x00;
							break;
						case 'E':
							frame[cmpt]=0x20;
							break;
					}
					cmpt++;
				}
			}
			valueEXToSend=valueEXToSend+maxvals;
			if (valueEXToSend>=nbValueEX) {
				valueEXToSend=0;
			}

		} else {
			frame[8]=sensorFrameName;
			bit9[8]=true;
			frame[9]=0;
			frame[9]|=(strlen(nameEX[sensorFrameName])<<3);
			frame[9]|=(strlen(unitEX[sensorFrameName])&0x03);
			bit9[9]=true;
			cmpt=10;
			for (i_Loop=0;i_Loop<strlen(nameEX[sensorFrameName]);i_Loop++) {
				frame[cmpt]=nameEX[sensorFrameName][i_Loop];
				bit9[cmpt]=true;
				cmpt++;
			}
			for (i_Loop=0;i_Loop<strlen(unitEX[sensorFrameName]);i_Loop++) {
				frame[cmpt]=unitEX[sensorFrameName][i_Loop];
				bit9[cmpt]=true;
				cmpt++;
			}
			sensorFrameName++;
			sensorFrameName%=nbValueEX;
		}
		// Finish the EX frame; //
		frame[2]|=cmpt-2; 
		//if (!(frame[2] & 0x02)) {frame[8]^=0x3F;}
		// Compute crc8 (last data);
		crc8=0;
			for (i_Loop=2; i_Loop<cmpt; i_Loop++) {
			crc8=pgm_read_byte(&(crctable[(crc8 ^ frame[i_Loop])]));}
		//crc8 = crc8fce(&frame[2],cmpt-2);
		frame[cmpt]=crc8;
		bit9[cmpt]=true;
		cmpt++;
	}
	// Send the value to print on jetiBox 2*16 characters
	frame[cmpt]=0xFE;
	bit9[cmpt]=false;
	cmpt++;
	for (i_Loop=0;i_Loop<LCDMaxPos;i_Loop++) {
	   frame[cmpt]=jetiLcd[i_Loop];
	   bit9[cmpt]=true;
	   cmpt++;
	}
	frame[cmpt]=0xFF;
	bit9[cmpt]=false;
	cmpt++;
	frameSize=cmpt;
	return true;
}

uint8_t JETI_Box_class::readbuttons() {
	if (lastbuttons != buttons) {
		return lastbuttons = buttons;
	} else {
		return JB_key_none;
	}
}

// Preinstantiate Objects //////////////////////////////////////////////////////
//JETI_Box_class JB;
