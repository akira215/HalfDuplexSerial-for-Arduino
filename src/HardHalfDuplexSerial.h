/*
HardHalfDuplexSerial.h
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
 Used to communicate using a built-in UART in Half Duplex mode.
 A direction pin should be set up using hardHalfDuplexSerial::setDirPin(const uint8_t dirPin);. This pin will be high when transmitting and low when receiving data.
 The library use TX Complete interrupt to switch the direction pin.
 The cpp file contains also the contructed instance hdSerial1, hdSerial2 & hdSerial3, depending on the hardware.
 An hdSerial could be constructed on port0 by uncommenting few lines.
*/

#ifndef HardHalfDuplexSerial_h
#define HardHalfDuplexSerial_h

#include <Arduino.h>
#include <HalfDuplexSerial.h>

#include <HardwareSerial.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#if !defined(__SAM3X8E__)
  // Ensure that the various bit positions we use are available with a 0
  // postfix, so we can always use the values for UART0 for all UARTs. The
  // alternative, passing the various values for each UART to the
  // HardwareSerial constructor also works, but makes the code bigger and
  // slower.
  #if !defined(TXC0)
  #if defined(TXC)
  // On ATmega8, the uart and its bits are not numbered, so there is no TXC0 etc.
  #define TXCIE0 TXCIE
  #elif defined(TXC1)
  // Some devices have uart1 but no uart0
  #define TXCIE0 TXCIE1
  #else
  #error No UART found in HardHalfDuplexSerial.h
  #endif
  #endif // !defined TXC0

  // Check at compiletime that it is really ok to use the bit positions of
  // UART0 for the other UARTs as well, in case these values ever get
  // changed for future hardware.
  #if defined(TXC1) && (TXCIE1 != TXCIE0)
  #error "HardHalfDuplex.h : Not all bit positions (TXCIE) for UART1 are the same as for UART0"
  #endif
  #if defined(TXC2) && (TXCIE2 != TXCIE0)
  #error "HardHalfDuplex.h : Not all bit positions (TXCIE) for UART2 are the same as for UART0"
  #endif
  #if defined(TXC3) && (TXCIE3 != TXCIE0)
  #error "HardHalfDuplex.h : Not all bit positions (TXCIE) for UART3 are the same as for UART0"
  #endif
#endif // !(__SAM3X8E__)


/******************************************************************************
* Definitions
******************************************************************************/
class hardHalfDuplexSerial : public halfDuplexSerial
{
private:
  HardwareSerial* _port;

#if defined  (__SAM3X8E__)
  Uart* _pUart;  // This is stored by USART class
  Pio*  _pioRTSReg;
  uint32_t _pinRTS;
#else
  volatile uint8_t * const _ucsrb;
  volatile uint8_t _dirBitMask;        // Pin used to swap in transmit/receive mode
  volatile uint8_t* _dirPortRegister;  // Port of the Pin used to swap in transmit/receive mode
#endif //  (__SAM3X8E__)

  bool _dirPinInitialized;

public:
  // public methods
#if defined  (__SAM3X8E__)
  hardHalfDuplexSerial (HardwareSerial* serial_port, Usart* pUsart, Pio* pioRTSReg, uint32_t pinRTS);
#else
  hardHalfDuplexSerial(HardwareSerial* serial_port, volatile uint8_t *ucsrb);
  void setDirPin(const uint8_t dirPin);
  inline void _tx_complete_irq() __attribute__((__always_inline__));
#endif //  (__SAM3X8E__)

  void begin(unsigned long baud);
  void end();

  virtual size_t write(uint8_t data);
  virtual size_t write(const uint8_t *buffer, /*size_t*/ int size);

  inline int read() { return _port->read();}
  inline void flush() {return _port->flush();}
  inline int available() {return _port->available();}

};

/*
#if defined (HAVE_HWSERIAL0) || (ID_UART)
  extern hardHalfDuplexSerial hdSerial;
#endif
*/
#if defined (HAVE_HWSERIAL1) || (ID_USART0)
  extern hardHalfDuplexSerial hdSerial1;
#endif

#if defined (HAVE_HWSERIAL2) || (ID_USART1)
  extern hardHalfDuplexSerial hdSerial2;
#endif
#if defined (HAVE_HWSERIAL2) || (ID_USART3)
  extern hardHalfDuplexSerial hdSerial3;
#endif


#endif  // HardHalfDuplexSerial_h
