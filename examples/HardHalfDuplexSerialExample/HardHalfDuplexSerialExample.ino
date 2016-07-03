/*
HalfDuplexSerialExample.ino
Written by Akira 

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
 This is a simple code to illustrate how to use SoftHalfDuplex lib
 Hardware just require a connection on the driver on pin 8. The device should be grounded with Arduino board.
 On Arduino Due Board, only corresponding USART RTS pin could be used to drive the tranceiver:
 - using hdSerial1 (Tx 18, Rx 19, Rts 2)
 - using hdSerial2 (Tx 16, Rx 17, Rts 23)
 - Serial3 RTS pin is not connected on Arduino Due Board

For more informations, please visit :
https://github.com/akira215/SoftHalfDuplexSerial-for-Arduino
*/

#include <HardHalfDuplexSerial.h>


byte readString[64];
int nByte;

void setup() {
  // Open serial communications and wait for port to open (PC communication)
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  // The following line is not used with Arduino Due board. Only corresponding USART RTS pin is used to drive the tranceiver
  hdSerial1.setDirPin(8); // hdSerial1 is an instance created in HardHalfDuplexSerial.cpp. Pin 8 will be used to switch direction
			
  hdSerial1.begin(57600); // Starting half Duplex communication at 57600 bps. 
  nByte = 0;
}

void loop() {
  while (Serial.available()) {
    readString[nByte] = Serial.read();  //gets one byte from serial buffer
    nByte += 1; // add one byte
  }

  hdSerial1.flushRx(); // flush the Rx buffer
  hdSerial1.write (readString, nByte); // send the read string on half duplex serial
                                      // alternatively, byte (char) could be sent one by one
                                      // using halfDuplexSerial::write(uint8_t byte)
 
  nByte = 0; // reset the readString
  delay(1000); // wait for the response
  
  while(hdSerial1.available())
  {
    readString[nByte] = hdSerial1.read();  // read from half duplex serial
    nByte += 1; // add one byte
  }

  for (int i=0; i<nByte; i++)
    Serial.print(readString[i]);  // print the result on UART

  Serial.println();  // go to new line on UART
}
