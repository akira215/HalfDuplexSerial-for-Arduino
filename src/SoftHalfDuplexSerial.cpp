/*
SoftHalfDuplexSerial.cpp (from SoftwareSerial lib)
 Written by Akira.

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

 *****************************************************************************
 Decription:
 Used to communicate using a software UART in Half Duplex mode, using only one pin of the Arduino Board.
 The receiver should obviously be grouded with the Arduino board (so at least 2 wires between the receiver and the Arduino)
 The pin shouldsupport change interrupts:
 - 0n the Mega and Mega 2560 only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
 - On the Leonardo and Micro only the following can be used for RX: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 If more than one soft serial port is used, halfDuplexSerial.listen() should be called on the port to be listened before receiving data.

For more informations, please visit :
https://github.com/akira215/SoftHalfDuplexSerial-for-Arduino
*/

#if !defined (__SAM3X8E__)
// When set, _DEBUG co-opts pins 11 and 13 for debugging with an
// oscilloscope or logic analyzer.  Beware: it also slightly modifies
// the bit times, so don't rely on it too much at high baud rates
#define _DEBUG 0
#define _DEBUG_PIN1 11
#define _DEBUG_PIN2 13
//
// Includes
//
#include <SoftHalfDuplexSerial.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <util/delay_basic.h>

//
// Statics
//
softHalfDuplexSerial *softHalfDuplexSerial::active_object = 0;
char softHalfDuplexSerial::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t softHalfDuplexSerial::_receive_buffer_tail = 0;
volatile uint8_t softHalfDuplexSerial::_receive_buffer_head = 0;

//
// Debugging
//
// This function generates a brief pulse
// for debugging or measuring on an oscilloscope.
#if _DEBUG
inline void DebugPulse(uint8_t pin, uint8_t count)
{
  volatile uint8_t *pport = portOutputRegister(digitalPinToPort(pin));

  uint8_t val = *pport;
  while (count--)
  {
    *pport = val | digitalPinToBitMask(pin);
    *pport = val;
  }
}
#else
inline void DebugPulse(uint8_t, uint8_t) {}
#endif

//
// Private methods
//

/* static */
inline void softHalfDuplexSerial::tunedDelay(uint16_t delay) {
  _delay_loop_2(delay);
}

// This function sets the current object as the "listening"
// one and returns true if it replaces another
bool softHalfDuplexSerial::listen()
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

// Stop listening. Returns true if we were actually listening.
bool softHalfDuplexSerial::stopListening()
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
void softHalfDuplexSerial::recv()
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

  uint8_t d = 0;

  // If RX line is high, then we don't see any start bit
  // so interrupt is probably not for us
  if (!rx_pin_read())
  {
    // Disable further interrupts during reception, this prevents
    // triggering another interrupt directly after we return, which can
    // cause problems at higher baudrates.
    setRxIntMsk(false);

    // Wait approximately 1/2 of a bit width to "center" the sample
    tunedDelay(_rx_delay_centering);

    DebugPulse(_DEBUG_PIN2, 1);

    // Read each of the 8 bits
    for (uint8_t i=8; i > 0; --i)
    {
      tunedDelay(_rx_delay_intrabit);
      d >>= 1;
      DebugPulse(_DEBUG_PIN2, 1);
      if (rx_pin_read())
        d |= 0x80;
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
      DebugPulse(_DEBUG_PIN1, 1);
      _buffer_overflow = true;
    }

    // skip the stop bit
    tunedDelay(_rx_delay_stopbit);
    DebugPulse(_DEBUG_PIN1, 1);

    // Re-enable interrupts when we're sure to be inside the stop bit
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

uint8_t softHalfDuplexSerial::rx_pin_read()
{
  return *_receivePortRegister & _dataBitMask;
}

//
// Interrupt handling
//

/* static */
inline void softHalfDuplexSerial::handle_interrupt()
{
  if (active_object)
  {
    active_object->recv();
  }
}

#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{
  softHalfDuplexSerial::handle_interrupt();
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

//
// Constructor
//
softHalfDuplexSerial::softHalfDuplexSerial(const uint8_t dataPin) :
 halfDuplexSerial(),
  _rx_delay_centering(0), _rx_delay_intrabit(0),
  _rx_delay_stopbit(0), _tx_delay(0),
  _buffer_overflow(false)
{
  _dataPin = dataPin;
  setRX();  // required to pull up the UART line

 // Initialize transmit register
 _dataBitMask = digitalPinToBitMask(_dataPin);
 uint8_t port = digitalPinToPort(_dataPin);
 _transmitPortRegister = portOutputRegister(port);
 _receivePortRegister = portInputRegister(port);

}

//
// Destructor
//
softHalfDuplexSerial::~softHalfDuplexSerial()
{
  end();
}

void softHalfDuplexSerial::setTX()
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output hihg. Now, it is input with pullup for a short while, which
  // is fine. UART line should be HIGH during idle state.
  digitalWrite(_dataPin,HIGH);
  pinMode(_dataPin, OUTPUT);
}

void softHalfDuplexSerial::setRX()
{
  // UART line should be HIGH during idle state.
  pinMode(_dataPin, INPUT_PULLUP);
  //pinMode(_dataPin, INPUT);
}

uint16_t softHalfDuplexSerial::subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

void softHalfDuplexSerial::setRxIntMsk(bool enable)
{
    if (enable)
      *_pcint_maskreg |= _pcint_maskvalue;
    else
      *_pcint_maskreg &= ~_pcint_maskvalue;
}

//
// Public methods
//

void softHalfDuplexSerial::begin(const unsigned long speed)
{
  halfDuplexSerial::begin(speed);
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  // Precalculate the various delays, in number of 4-cycle delays
  uint16_t bit_delay = (F_CPU / speed) / 4;

  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  _tx_delay = subtract_cap(bit_delay, 15 / 4);

  // Only setup rx when we have a valid PCINT for this pin
  if (digitalPinToPCICR(_dataPin)) {
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
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);
    #else // Timings counted from gcc 4.3.2 output
    // Note that this code is a _lot_ slower, mostly due to bad register
    // allocation choices of gcc. This works up to 57600 on 16Mhz and
    // 38400 on 8Mhz.
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
    _rx_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4);
    #endif


    // Enable the PCINT for the entire port here, but never disable it
    // (others might also need it, so we disable the interrupt by using
    // the per-pin PCMSK register).
    *digitalPinToPCICR(_dataPin) |= _BV(digitalPinToPCICRbit(_dataPin));
    // Precalculate the pcint mask register and value, so setRxIntMask
    // can be used inside the ISR without costing too much time.
    _pcint_maskreg = digitalPinToPCMSK(_dataPin);
    _pcint_maskvalue = _BV(digitalPinToPCMSKbit(_dataPin));

    tunedDelay(_tx_delay); // if we were low this establishes the end
  }

#if _DEBUG
  pinMode(_DEBUG_PIN1, OUTPUT);
  pinMode(_DEBUG_PIN2, OUTPUT);
#endif

  listen();
}

void softHalfDuplexSerial::end()
{
  stopListening();
}


// Read data from buffer
int softHalfDuplexSerial::read()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head]; // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int softHalfDuplexSerial::available()
{
  if (!isListening())
    return 0;

  return (_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head) % _SS_MAX_RX_BUFF;
}

size_t softHalfDuplexSerial::write(uint8_t b)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }
  uint8_t result = b;

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile uint8_t *reg = _transmitPortRegister;
  uint8_t reg_mask = _dataBitMask;
  uint8_t inv_mask = ~_dataBitMask;
  uint16_t delay = _tx_delay;
  uint8_t oldSREG = SREG; // save interrupt flag


  cli();  // turn off interrupts for a clean txmit

  setTX();   // Changing dataPin to transmit

  // Write the start bit
  *reg &= inv_mask;

  tunedDelay(delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
      *reg |= reg_mask; // send 1
    else
      *reg &= inv_mask; // send 0

    tunedDelay(delay);
    b >>= 1;
  }

  // restore pin to natural state
  *reg |= reg_mask;

  setRX();   // Changing dataPin to receive again

  SREG = oldSREG; // turn interrupts back on

  return result;
  // return 1;
}

size_t softHalfDuplexSerial::write(const uint8_t *buffer, /*size_t*/ int size)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile uint8_t *reg = _transmitPortRegister;
  uint8_t reg_mask = _dataBitMask;
  uint8_t inv_mask = ~_dataBitMask;
  uint16_t delay = _tx_delay;
  uint8_t b;  // buffer for 1 byte
  uint8_t oldSREG = SREG; // save interrupt flag

  cli();  // turn off interrupts for a clean txmit

  setTX();   // Changing dataPin to transmit

  for (int j = 0; j < size; j++)
  {

    b = buffer[j]; // load 1 byte buffer

    // Write the start bit
    *reg &= inv_mask;

    tunedDelay(delay);

    // Write each of the 8 bits
    for (uint8_t i = 8; i > 0; --i)
    {
      if (b & 1) // choose bit
        *reg |= reg_mask; // send 1
      else
        *reg &= inv_mask; // send 0

      tunedDelay(delay);
      b >>= 1;
    }

    // restore pin to natural state
    *reg |= reg_mask;
    tunedDelay(delay);  // this act for the stop bit
  }


  setRX();   // Changing dataPin to receive again
  SREG = oldSREG; // turn interrupts back on
  // tunedDelay(_tx_delay);



  return size;
}

void softHalfDuplexSerial::flush()
{
  // There is no tx buffering, simply return
}

void softHalfDuplexSerial::flushRx()
{
  // simply reinitialize the buffer
  _receive_buffer_head = _receive_buffer_tail = 0;

}


int softHalfDuplexSerial::peek()
{
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}

#endif  // (__SAM3X8E__)
