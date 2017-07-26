#if defined(__AVR_ATmega328P__)

// disable the stock Arduino serial driver
#ifdef HardwareSerial_h
# error Must include disableserial.h before the Arduino serial driver is defined.
#endif
#define HardwareSerial_h


#endif

