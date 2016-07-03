/*
SoftHalfDuplexSerial.h (from SoftwareSerial lib)
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
 The pin should support change interrupts:
 - 0n the Mega and Mega 2560 only the following can be used for RX: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8 (62), A9 (63), A10 (64), A11 (65), A12 (66), A13 (67), A14 (68), A15 (69).
 - On the Leonardo and Micro only the following can be used for RX: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 If more than one soft serial port is used, halfDuplexSerial.listen() should be called on the port to be listened before receiving data.

For more informations, please visit :
https://github.com/akira215/SoftHalfDuplexSerial-for-Arduino
*/

#ifndef SoftHalfDuplexSerial_h
#define SoftHalfDuplexSerial_h

#if !defined (__SAM3X8E__)
#include <HalfDuplexSerial.h>
#include <inttypes.h>
#include <Stream.h>

/******************************************************************************
* Definitions
******************************************************************************/

#define _SS_MAX_RX_BUFF 64 // RX buffer size
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class softHalfDuplexSerial : public Stream,  public halfDuplexSerial
{
private:
  // per object data
  uint8_t _dataPin;
  uint8_t _dataBitMask;
  volatile uint8_t *_transmitPortRegister;
  volatile uint8_t *_receivePortRegister;
  volatile uint8_t *_pcint_maskreg;
  uint8_t _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;
  uint16_t _tx_delay;

  uint16_t _buffer_overflow:1;
  //uint16_t _inverse_logic:1;

  // static data
  static char _receive_buffer[_SS_MAX_RX_BUFF];
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static softHalfDuplexSerial *active_object;

  // private methods
  inline void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();
  void setTX(/*uint8_t transmitPin*/);
  void setRX(/*uint8_t receivePin*/);
  inline void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);

  // private static method for timing
  static inline void tunedDelay(uint16_t delay);

public:
  // public methods
  softHalfDuplexSerial(const uint8_t dataPin);
  ~softHalfDuplexSerial();
  virtual void begin(const unsigned long speed);
  bool listen();
  void end();
  bool isListening() { return this == active_object; }
  bool stopListening();
  bool overflow() { bool ret = _buffer_overflow; if (ret) _buffer_overflow = false; return ret; }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, /*size_t*/ int size);
  virtual int read();
  virtual int available();
  virtual void flush();
  virtual void flushRx();
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

#endif  // (__SAM3X8E__)
#endif  // SoftHalfDuplexSerial_h
