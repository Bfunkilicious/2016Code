/*
 * lidar.h
 *
 *  Created on: Jan 21, 2016
 *      Author: tigertronics
 */

#ifndef SRC_LIDAR_LIDAR_H_
#define SRC_LIDAR_LIDAR_H_

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

#include "I2C.h"

class LIDARClass {
public:
	void LIDARinit();
	float getDistance();
	void loop();
private:
	I2C* lidarptr;
	unsigned char distanceArray[2]; // array to store distance bytes from read function
	int nackack;
	int distance;
};
#endif /* SRC_LIDAR_LIDAR_H_ */
