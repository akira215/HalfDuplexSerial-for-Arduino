/*
HardHalfDuplexSerial.cpp
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
#include <Arduino.h>
#include <HardHalfDuplexSerial.h>


// Actual interrupt handlers //////////////////////////////////////////////////////////////

inline void hardHalfDuplexSerial::_tx_complete_irq()
{
  // if we are transmitting, the data has been sent, clear the flag and release the dirPin line
  if (_isTransmitting)
  {
    *_dirPortRegister &= ~_dirBitMask; //digitalWrite(_dirPin, LOW);// turn off the dir pin (dir pin)
    _isTransmitting = false;
  }
}

//
// Constructor
//
hardHalfDuplexSerial::hardHalfDuplexSerial(HardwareSerial* serial_port, volatile uint8_t *ucsrb) :
  halfDuplexSerial(),
  _port(serial_port), _ucsrb(ucsrb), _dirBitMask(0),
 _dirPortRegister(0), _dirPinInitialized(false)
{
}

void hardHalfDuplexSerial::setDirPin(const uint8_t dirPin)
{
  _dirBitMask = digitalPinToBitMask(dirPin); // bit of dir pin
  _dirPortRegister = portOutputRegister(digitalPinToPort(dirPin)); // Output register of dir pin
  
  pinMode(dirPin, OUTPUT);
  *_dirPortRegister &= ~_dirBitMask; // digitalWrite(dirPin, LOW);  // should be put to High when writting
  _dirPinInitialized = true;
 
}


void hardHalfDuplexSerial::begin(unsigned long baud)
{
  if(_dirPinInitialized)
  {
    halfDuplexSerial::begin(baud);
    _port->begin(baud);

    sbi( *_ucsrb, TXCIE0); // Setup the Tx Complete interrupt
  }
}

void hardHalfDuplexSerial::end()
{
    _port->end();
    cbi( *_ucsrb, TXCIE0);    // Clear the Tx Complete interrupt
}

size_t hardHalfDuplexSerial::write(uint8_t data)
{
  *_dirPortRegister |=_dirBitMask; //digitalWrite(_dirPin, HIGH);  should be put to High when writting (dir pin)
  
  _port->write(data);
  _isTransmitting = true;
  return 1;
}

size_t hardHalfDuplexSerial::write(const uint8_t* buffer,  int size)
{
  *_dirPortRegister |=_dirBitMask; //digitalWrite(_dirPin, HIGH);  should be put to High when writting (dir pin)

  size_t nByteSent = _port->write(buffer,size);
  _isTransmitting = true;

  return nByteSent;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    Creating instances
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
#ifdef HAVE_HWSERIAL0
  #if defined(USART_TX_vect)
    ISR(USART_TX_vect)
  #elif defined(USART0_TX_vect)
    ISR(USART0_TX_vect)
  #elif defined(USART_TXC_vect)
    ISR(USART_TXC_vect) // ATmega8
  #else
    #error "Don't know what the Data Received vector is called for Serial"
  #endif
  {
    hdSerial._tx_complete_irq();
  }
  // Creating the instance of the class for serial0
  #if defined(UBRRH) && defined(UBRRL)
  	hardHalfDuplexSerial hdSerial(Serial, &UCSRB);
  #else
  	hardHalfDuplexSerial hdSerial(Serial, &UCSR0B);
  #endif
#endif // HAVE_HWSERIAL0
*/

#ifdef SERIAL1
  #warning ok serial1
#endif

#ifdef HAVE_HWSERIAL1
  #if defined(UART1_TX_vect)
    ISR(UART1_TX_vect)
  #elif defined(USART1_TX_vect)
    ISR(USART1_TX_vect)
  #else
    #error "Don't know what the Transmit Complete vector is called for Serial1"
  #endif
  {
    hdSerial1._tx_complete_irq();
  }

  // Creating the instance of the class for serial1
  hardHalfDuplexSerial hdSerial1(&Serial1, &UCSR1B );
#endif // HAVE_HWSERIAL1

#ifdef HAVE_HWSERIAL2
  #if defined(UART2_TX_vect)
    ISR(UART2_TX_vect)
  #elif defined(USART2_TX_vect)
    ISR(USART2_TX_vect)
  #else
    #error "Don't know what the Transmit Complete vector is called for Serial2"
  #endif
  {
    hdSerial2._tx_complete_irq();
  }
  // Creating the instance of the class for serial2
  hardHalfDuplexSerial hdSerial2(&Serial2, &UCSR2B); 
#endif // HAVE_HWSERIAL2

#ifdef HAVE_HWSERIAL3
  #if defined(UART3_TX_vect)
    ISR(UART3_TX_vect)
  #elif defined(USART3_TX_vect)
    ISR(USART3_TX_vect)
  #else
    #error "Don't know what the Transmit Complete vector is called for Serial3"
  #endif
  {
    hdSerial3._tx_complete_irq();
  }
  // Creating the instance of the class for serial3
  hardHalfDuplexSerial hdSerial3(&Serial3, &UCSR3B);
#endif // HAVE_HWSERIAL3


