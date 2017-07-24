#ifndef JetiCfg_h
#define JetiCfg_h


#define JETI_REPEAT_ALARM 6

//send one value definition each second so all sensor are listed in about 20s
#define JETI_HEADERTIME 1000

//minimum time reported between two key hits on same key
#define JETI_LASTKEYTIME 500

#if defined (__AVR_ATtiny85__)

#define JETI_EX_MAXVALUES 3
#define JETI_14 0
#define JETI_6 0
#define JETI_DateTime 0
#define JETI_GPS 0
#else

#define JETI_EX_MAXVALUES 16
#define JETI_14 1
#define JETI_6 1
#define JETI_DateTime 1
#define JETI_GPS 1


#endif




#endif
