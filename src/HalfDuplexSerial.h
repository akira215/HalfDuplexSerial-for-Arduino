/*
 HalfDuplexSerial.h
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
 halfDuplexSerial is an abstract class used to communicate serial data in half duplex mode.
 This base class comes with to implemantation :
  - hardHalfDuplexSerial used Arduino built-in UART to communicate. A direction pin is used to switch between receive & transmit mode
  - softHalfDuplexSerial used change interrupts to communicate with only one pin.
*/

#ifndef HalfDuplexSerial_h
#define HalfDuplexSerial_h

#include <Arduino.h>

/******************************************************************************
* Definitions
******************************************************************************/

class halfDuplexSerial
{
public:
  // public methods
  halfDuplexSerial();

  virtual void begin(unsigned long baud);
  virtual void end() = 0;

  virtual size_t write(const uint8_t data);
  virtual size_t write(const uint8_t *buffer, /*size_t*/ int size);

  virtual int read()=0;                 // Read one byte
  virtual void flush()=0;               // flush Tx buffer
  virtual void flushRx();
  virtual int available()=0;

  bool waitByteToReceived(const uint8_t nBytes);  // return false if a timeout occured
  
  inline bool isTransmitting() { return _isTransmitting; }

protected:
  unsigned int _delayForOneByte;       // Delay in µs to transmit/receive one byte
  volatile bool _isTransmitting;       // Should be true during transmitting and false as soon as transmit is complete
 

};


#endif  // HalfDuplexSerial_h
