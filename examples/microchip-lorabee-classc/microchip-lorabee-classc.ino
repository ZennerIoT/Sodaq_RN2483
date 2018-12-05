/*
* Copyright (c) 2015 SODAQ. All rights reserved.
*
* This file is part of Sodaq_RN2483.
*
* Sodaq_RN2483 is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of
* the License, or(at your option) any later version.
*
* Sodaq_RN2483 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with Sodaq_RN2483.  If not, see
* <http://www.gnu.org/licenses/>.
*/

#include <Sodaq_RN2483.h>

#if defined(ARDUINO_AVR_SODAQ_MBILI) || defined(ARDUINO_AVR_SODAQ_TATU)
// MBili
#define debugSerial Serial
#define loraSerial Serial1
#define beePin 20
#elif defined(ARDUINO_SODAQ_AUTONOMO)
// Autonomo
#define debugSerial SerialUSB
#define loraSerial Serial1
#define beePin BEE_VCC
#elif defined(ARDUINO_SODAQ_ONE) || defined(ARDUINO_SODAQ_ONE_BETA)
// Sodaq One
#define debugSerial SerialUSB
#define loraSerial Serial1
#elif defined(ARDUINO_SODAQ_EXPLORER)
#define debugSerial SerialUSB
#define loraSerial Serial2
#else
#error "Please select Autonomo, Mbili, or Tatu"
#endif

const uint8_t devAddr[4] =
{
	0xAB, 0xAB, 0xAB, 0xAB
};

// USE YOUR OWN KEYS!
const uint8_t appSKey[16] =
{
	0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA,
	0xAA, 0xAA, 0xAA, 0xAA,
};

// USE YOUR OWN KEYS!
const uint8_t nwkSKey[16] =
{
	0xBD, 0xBD, 0xBD, 0xBD,
	0xBD, 0xBD, 0xBD, 0xBD,
	0xBD, 0xBD, 0xBD, 0xBD,
	0xBD, 0xBD, 0xBD, 0xBD,
};

uint8_t testPayload[] =
{
	0x30, 0x31, 0xFF, 0xDE, 0xAD
};

void onReceive(const uint8_t* buffer, uint16_t size){
  debugSerial.print("GOT DOWNLINK:");
  for(uint16_t i;i<size;i++){
    debugSerial.print(buffer[i],HEX);
    debugSerial.print(", ");
  }
  debugSerial.println();
}

void setup()
{
  while ((!SerialUSB) && (millis() < 10000)){
    // Wait 10 seconds for the Serial Monitor
  }
  
  //Power up the LoRaBEE
  #if defined(ARDUINO_AVR_SODAQ_MBILI) || defined(ARDUINO_SODAQ_AUTONOMO)
  pinMode(BEE_VCC, OUTPUT);
  digitalWrite(BEE_VCC, HIGH);
  #endif
  
	debugSerial.begin(115200);
	loraSerial.begin(LoRaBee.getDefaultBaudRate());


	//LoRaBee.setDiag(debugSerial); // optional
	if (LoRaBee.initABP(loraSerial, devAddr, appSKey, nwkSKey, true))
	{
		debugSerial.println("Connection to the network was successful.");
	}
	else
	{
		debugSerial.println("Connection to the network failed!");
	}

  LoRaBee.sendCommand("mac set class c");
  LoRaBee.sendReqAck(1, testPayload, 5, 3);
  LoRaBee.setReceiveCallback(onReceive);
}

void loop()
{
	LoRaBee.waitRx(0);
}
