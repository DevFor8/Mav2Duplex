/*
SoftwareSerialO2.cpp
- Modification of SoftSer to work with 2 stop bits and Odd parity
- timing is also precisioned for Arduino Pro Mini 5V/16Mhz with 328 processor
- Modifications by DevFor8.com - info@devfor8.com
Additionally, for small devices such as ATtiny85 (Digispark), it's possible to declare the same pin for TX and RX.
Data direction is set by using the new txMode() and rxMode() methods.
RC Navy (2012-2015): http://p.loussouarn.free.fr

SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
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

// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
//#define _DEBUG 1
#define _DEBUG_PIN1 0
#define _DEBUG_PIN2 1
//#define FAST_DEBUG //less intrusive

// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "SoftwareSerialO2.h"
#include <util/delay_basic.h>
//
// Statics
//
SoftwareSerial *SoftwareSerial::active_object = 0;
unsigned char SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF];
uint16_t SoftwareSerial::bit9buff = 0;


volatile unsigned char SoftwareSerial::_receive_buffer_tail = 0;
volatile unsigned char SoftwareSerial::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
#if _DEBUG
#if defined(FAST_DEBUG)
#define DebugPulse(a, bit) sbi(PINB, bit)
#else
inline void DebugPulse(unsigned char pin, unsigned char count)
{
  volatile unsigned char *pport = portOutputRegister(digitalPinToPort(pin));

  unsigned char val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
}
#endif
#else
// no debug
inline void DebugPulse(unsigned char pin, unsigned char count)
{
}
#endif
//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t delay) { 
  _delay_loop_2(delay);
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another 
bool SoftwareSerial::listen()
{
  if (!_rx_delay_stopbit)
    return false;
  if (active_object != this)
  {
    if (active_object)
      active_object->stopListening();
    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    setRxIntMsk(true);
    return true;
  }

  return false;
}

/*bool SoftwareSerial::overflow() 
{ 
  bool ret = _buffer_overflow; _buffer_overflow = false; return ret; 
 }
 */


// Stop listening. Returns true if we were actually listening.
bool SoftwareSerial::stopListening()
{
  if (active_object == this)
  {
    setRxIntMsk(false);
    active_object = NULL;
    return true;
  }
  return false;
}
//
// The receive routine called by the interrupt handler
//
void SoftwareSerial::recv()
{

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Preserve the registers that the compiler misses
// (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t"
    ::);
#endif  

  unsigned char d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if ( !rx_pin_read())
  {

    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);
    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (unsigned char i=0x1; i; i <<= 1)
    {
      tunedDelay(_rx_delay_intrabit);
      DebugPulse(_DEBUG_PIN2, 1);
      unsigned char noti = ~i;
      if (rx_pin_read())
        d |= i;
      else // else clause added to ensure function timing is ~balanced
        d &= noti;
    }

    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head)
    {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d; // save new byte
      _receive_buffer_tail = next;
    } 
    else 
    {

#if _DEBUG // for scope: pulse pin as overflow indictator
      DebugPulse(_DEBUG_PIN1, 1);
#endif
      _buffer_overflow = true;
    }
    // skip the parity & 2x stop bit
    if (is9bit)
       {
        tunedDelay(_rx_delay_intrabit); //9th bit
        bitSet(bit9buff,rx_pin_read());
       }
        
      
    if (_rx_delay_paritybit != 0)
       tunedDelay(_rx_delay_paritybit);
    tunedDelay(_rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN2, 1);
    setRxIntMsk(true);

  }

#if GCC_VERSION < 40302
// Work-around for avr-gcc 4.3.0 OSX version bug
// Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t"
    ::);
#endif
}
unsigned char SoftwareSerial::rx_pin_read()
{
  return *_receivePortRegister & _receiveBitMask;
}

//
// Interrupt handling
//

/* static */
inline void SoftwareSerial::handle_interrupt()
{
    
  if (active_object)
  {
    active_object->recv();
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  SoftwareSerial::handle_interrupt();
}
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
#endif

void SoftwareSerial::tx_pin_write(unsigned char pin_state)
{
  if (pin_state == LOW)
    *_transmitPortRegister &= ~_transmitBitMask;
  else
    *_transmitPortRegister |= _transmitBitMask;
}


//
// Constructor
//


SoftwareSerial::SoftwareSerial(signed char receivePin, signed char transmitPin) : 
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _buffer_overflow(false)
{
  if (transmitPin>=0)
    setTX(transmitPin);
  if (receivePin>=0)
    setRX(receivePin);
}


void SoftwareSerial::Init(signed char receivePin, signed char transmitPin)
{
    if (transmitPin>=0)
setTX(transmitPin);
   if (receivePin>=0)
 setRX(receivePin);
}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(unsigned char tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output high. Now, it is input with pullup for a short while, which
  // is fine. With inverse logic, either order is fine.
  digitalWrite(tx, HIGH);
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
   _transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial::setRX(unsigned char rx)
{
  pinMode(rx, INPUT);
    digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  unsigned char port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

uint16_t SoftwareSerial::subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

//
// Public methods
//

void SoftwareSerial::begin(long speed,bool is9, unsigned char stopbit, unsigned char paritybit)
{
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay= 0;
is9bit = is9;
  // Precalculate the various delays, in number of 4-cycle delays
  uint16_t bit_delay = (F_CPU / speed) / 4;

  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  _tx_delay = subtract_cap(bit_delay, 15 / 4);

  // Only setup rx when we have a valid PCINT for this pin
  if (digitalPinToPCICR(_receivePin)) {
    #if GCC_VERSION > 40800
    // Timings counted from gcc 4.8.2 output. This works up to 115200 on
    // 16Mhz and 57600 on 8Mhz.
    //
    // When the start bit occurs, there are 3 or 4 cycles before the
    // interrupt flag is set, 4 cycles before the PC is set to the right
    // interrupt vector address and the old PC is pushed on the stack,
    // and then 75 cycles of instructions (including the RJMP in the
    // ISR vector table) until the first delay. After the delay, there
    // are 17 more cycles until the pin value is read (excluding the
    // delay in the loop).
    // We want to have a total delay of 1.5 bit time. Inside the loop,
    // we already wait for 1 bit time - 23 cycles, so here we wait for
    // 0.5 bit time - (71 + 18 - 22) cycles.
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);

    // There are 23 cycles in each loop iteration (excluding the delay)
    _rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);

    // There are 37 cycles from the last bit read to the start of
    // stopbit delay and 11 cycles from the delay until the interrupt
    // mask is enabled again (which _must_ happen during the stopbit).
    // This delay aims at 3/4 of a bit time, meaning the end of the
    // delay will be at 1/4th of the stopbit. This allows some extra
    // time for ISR cleanup, which makes 115200 baud at 16Mhz work more
    // reliably
    _rx_delay_stopbit = ( subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4) ) + (bit_delay * stopbit );
    #else // Timings counted from gcc 4.3.2 output
    // Note that this code is a _lot_ slower, mostly due to bad register
    // allocation choices of gcc. This works up to 57600 on 16Mhz and
    // 38400 on 8Mhz.
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
    _rx_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
    _rx_delay_stopbit = ( subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4) )  + (bit_delay * stopbit) );
    #endif

    _rx_delay_paritybit = _rx_delay_intrabit * paritybit;



    // Enable the PCINT for the entire port here, but never disable it
    // (others might also need it, so we disable the interrupt by using
    // the per-pin PCMSK register).
      *digitalPinToPCICR(_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
    // Precalculate the pcint mask register and value, so setRxIntMask
    // can be used inside the ISR without costing too much time.
    _pcint_maskreg = digitalPinToPCMSK(_receivePin);
    _pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));

    tunedDelay(_tx_delay); // if we were low this establishes the end
  }

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  listen();
}

void SoftwareSerial::setRxIntMsk(bool enable)
{
    if (enable)
      *_pcint_maskreg |= _pcint_maskvalue;
    else
      *_pcint_maskreg &= ~_pcint_maskvalue;
}


void SoftwareSerial::end()
{
 stopListening();
}


// Read data from buffer
int SoftwareSerial::read()
{
  if (!isListening())
    {
      return -1;
    }

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    {
      return -1;
    }

  // Read from "head"
  unsigned char d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;


  return d;
}

// Read data from buffer
unsigned char SoftwareSerial::read9(bool* is9set)
{
  if (!isListening())
    {
      return -1;
    }

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    {
      return -1;
    }

  // Read from "head"
  unsigned char d = _receive_buffer[_receive_buffer_head]; // grab next byte
  *is9set = bitRead(bit9buff,_receive_buffer_head);
  
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;


  return d;
}

int SoftwareSerial::available()
{
  if (!isListening())
    {
    return 0;
    }

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(unsigned char b)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

#ifndef ALLOW_ISR
  unsigned char oldSREG = SREG;
  cli();  // turn off interrupts for a clean txmit
#endif

    unsigned char p = 0;

	
if (_rx_delay_paritybit != 0)
{
  for (unsigned char mask = 0x01; mask; mask <<= 1)
      if (b & mask) p++;
  if (set9bit) p++;
  p = ~p; //ODD
}

  // Write the start bit
  tx_pin_write(LOW);
  tunedDelay(_tx_delay);
  

  // Write each of the 8 bits
    for (unsigned char mask = 0x01; mask; mask <<= 1)
    {
      if (b & mask) // choose bit
        tx_pin_write(HIGH); // send 1
      else
        tx_pin_write(LOW); // send 0
    
      tunedDelay(_tx_delay);
    }
if (is9bit)
{
      if (set9bit) // choose bit
        tx_pin_write(HIGH); // send 1
      else
        tx_pin_write(LOW); // send 0

	  tunedDelay(_tx_delay);
}

		// parity
 if (_rx_delay_paritybit != 0)
{
    if (p & 0x01)
      tx_pin_write(HIGH); // send 1
    else 
      tx_pin_write(LOW); // send 0

      tunedDelay(_rx_delay_paritybit);
}
	  tx_pin_write(HIGH); // restore pin to natural state

#ifndef ALLOW_ISR  
  SREG = oldSREG; // turn interrupts back on
#endif

  tunedDelay(_rx_delay_stopbit); //<< is for 2 stop bits
  
  
  return 1;
}

void SoftwareSerial::flush()
{
  // There is no tx buffering, simply return
}

int SoftwareSerial::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

/* RC Navy: hack to use SofSerial as single wire bidirectional serial port */
void SoftwareSerial::txMode()
{
  /* Disable Pin Change Interrupt capabilities for this pin */
	active_object = 0;
  /* Switch Pin to Output */
  pinMode(_receivePin, OUTPUT);
  digitalWrite(_receivePin, HIGH);
}

void SoftwareSerial::rxMode()
{
  /* Enable Pin Change Interrupt capabilities for this pin */
	//active_object = this;

  /* Switch Pin to Input */
  pinMode(_receivePin, INPUT);
    digitalWrite(_receivePin, HIGH);  // pullup for normal logic!

  listen();
}

