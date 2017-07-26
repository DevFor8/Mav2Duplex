/*
<SoftSerial> library is exactly the same as the <SoftwareSerial> library but used with the <TinyPinChange> library which allows to share
- Modification of SoftSer to work with 2 stop bits and Odd parity
- timing is also precisioned for Arduino Pro Mini 5V/16Mhz with 328 processor
- Modifications by DevFor8.com - info@devfor8.com
RC Navy (2012-2015): http://p.loussouarn.free.fr


SoftwareSerial.h (formerly NewSoftSerial.h) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

The latest version of this library can always be found at
http://arduiniana.org.

   2014, by DevFor8.com, info@devfor8.com
   9 data, Odd parity, 2 stop modification

   2017, by Radek Voltr, voltr@voltr.eu
   single wire conversion, memory savings
   
*/

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include "ssconfig.h"
#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/
#define _SS_MAX_RX_BUFF 16 // RX buffer size, must follow 9bits length

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class SoftwareSerial : public Stream
{
private:
  // per object data
  unsigned char _receivePin;
  unsigned char _receiveBitMask;
  volatile unsigned char *_receivePortRegister;
  unsigned char _transmitBitMask;
  volatile unsigned char *_transmitPortRegister;
  volatile unsigned char *_pcint_maskreg;
  unsigned char _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)

  bool is9bit;

  uint16_t _buffer_overflow:1;

//#if defined (__AVR_ATmega328P__) 
  // static data
  static unsigned char _receive_buffer[_SS_MAX_RX_BUFF];
  static volatile unsigned char _receive_buffer_tail;
  static volatile unsigned char _receive_buffer_head;
  static uint16_t bit9buff;

//#endif
static SoftwareSerial *active_object;

  // private methods
//#if defined (__AVR_ATmega328P__) 
  inline void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();
//#endif
void tx_pin_write(unsigned char pin_state);
  void setTX(unsigned char transmitPin);
  void setRX(unsigned char receivePin);
  inline void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
	bool set9bit;

  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
 
  uint16_t _rx_delay_stopbit;
  uint16_t _rx_delay_paritybit;
  uint16_t _tx_delay;
  
  // public methods
  SoftwareSerial(signed char receivePin, signed char transmitPin);
  void Init(signed char receivePin, signed char transmitPin);

  ~SoftwareSerial();
  void begin(long speed, bool is9 = false, unsigned char stopbit = 1, unsigned char paritybit = 0);
//#if defined (__AVR_ATmega328P__) 
  bool listen();
  bool isListening() { return this == active_object; }
  bool stopListening();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  void rxMode();
//#endif
  void end();
  //bool overflow();
  void txMode();
  virtual size_t write(unsigned char byte);
  virtual void flush();
  virtual int read();
  unsigned char read9(bool* is9set);

  virtual int available();
  int peek();
  
operator bool() { return true; }
  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round

#endif
