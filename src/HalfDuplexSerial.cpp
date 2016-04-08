/*
 HalfDuplexSerial.cpp
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

#include <Arduino.h>
#include <HalfDuplexSerial.h>

//
// Constructor
//
halfDuplexSerial::halfDuplexSerial() :
  _delayForOneByte(0), _isTransmitting(false)
{
}


void halfDuplexSerial::begin(unsigned long baud)
{
   _delayForOneByte = (90000000 / baud)   ; // safety try to adjust this value.Theoretically : 1 000 000 µs * 10 (8 bit + 1 start + 1 stop) rounded up
}


size_t halfDuplexSerial::write(__attribute__((unused)) const uint8_t data)
{
  _isTransmitting = true;
  return 1;
}

size_t halfDuplexSerial::write(const uint8_t* buffer,  int size)
{
   
  size_t nByteSent = 0;
  for(int i=0; i<size; i++)
    if (write(buffer[i]))
      nByteSent++;
 
  _isTransmitting = true;

  return nByteSent;
}

bool halfDuplexSerial::waitByteToReceived(const uint8_t nBytes)
{
   unsigned long timeout = micros() + _delayForOneByte * nBytes;
   
  while(available() < nBytes) // the first start byte is read here
    if(micros()>timeout)
        return false;

  return true;
}

void halfDuplexSerial::flushRx()
{
  while(available())
    read();
}




