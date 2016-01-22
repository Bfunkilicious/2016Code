/*
 * lidar.cpp
 *
 *  Created on: Jan 21, 2016
 *      Author: tigertronics
 */

#include "lidar.h"

void LIDARClass::LIDARinit() {
	lidarptr = new I2C(I2C::Port::kOnboard, LIDARLite_ADDRESS);
	if(lidarptr->AddressOnly()) {
		printf("LIDAR connected!");
	}
	else {
		printf("LIDAR not connected!");
	}
	nackack = 100;
	distance = 0;
}

float LIDARClass::getDistance() {
	return distance;
}

void LIDARClass::loop() {
	uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses
	 while (nackack != 0) { // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
		 nackack = lidarptr->Write(LIDARLite_ADDRESS,RegisterMeasure); // Write 0x04 to 0x00
	     //usleep(1); // Wait 1 ms to prevent overpolling
	 }

	 nackack = 100;

	  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
	    nackack = lidarptr->Read(LIDARLite_ADDRESS,RegisterHighLowB, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
	    //usleep(1); // Wait 1 ms to prevent overpolling
	  }

	  distance = (distanceArray[0] << 8) + distanceArray[1]; // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
}
