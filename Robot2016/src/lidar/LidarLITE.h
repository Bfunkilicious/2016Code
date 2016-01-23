/*
 * LidarLITE.h
 *
 *  Created on: Jan 22, 2016
 *      Author: tigertronics
 */

#ifndef SRC_LIDAR_LIDARLITE_H_
#define SRC_LIDAR_LIDARLITE_H_

#include "WPILib.h"
#include <cstdint>
#include <unistd.h>

class LidarLITE {
public:
	LidarLITE();
	virtual ~LidarLITE();
	void getDistance();
private:
	I2C* lidarptr;
	enum Address {ADDRESS_DEFAULT=0x62}; // default I2C bus address for the LIDAR Lite v2
	enum Register {COMMAND=0x00, STATUS=0x01, DISTANCE_1_2=0x8f};
	enum Command {ACQUIRE_DC_CORRECT=0x04};
	enum NumberOfRegistersToRead {READ_1_REGISTER=0x01, READ_2_REGISTERS=0x02};
	enum NumberOfRegistersToWrite {WRITE_1_REGISTER=0x01};

	bool Busy()
	{
		unsigned char Status[LidarLITE::READ_1_REGISTER];
		unsigned char statusRegister[LidarLITE::WRITE_1_REGISTER];
		statusRegister[LidarLITE::WRITE_1_REGISTER-1] = LidarLITE::STATUS;

		/**********read status**********/
		if ( lidarptr->WriteBulk(statusRegister, LidarLITE::WRITE_1_REGISTER)) {printf ( "WriteBulk status failed! line %d\n", __LINE__ ); return true;}
		if ( lidarptr->ReadOnly(LidarLITE::READ_1_REGISTER, Status) ) {printf ( "ReadOnly status failed! line %d\n", __LINE__ ); return true;}
		//printf("Status at line %d %0x, bit0=%0x\n", __LINE__, Status[0], Status[0] & (unsigned char)0x01);
		return (Status[0] & (unsigned char)0x01); // bit 0 is LIDAR Lite v2 busy bit
	};
};

#endif /* SRC_LIDAR_LIDARLITE_H_ */
