/*
 * LidarLITE.cpp
 *
 *  Created on: Jan 22, 2016
 *      Author: tigertronics
 */

#include <lidar/LidarLITE.h>

LidarLITE::LidarLITE() {
	lidarptr = new I2C(I2C::Port::kOnboard, LidarLITE::ADDRESS_DEFAULT);
	if(lidarptr->AddressOnly()) {
		printf("LidarLITE connected!\n");
	}
	else {
		printf("LidarLITE could not connect!\n");
	}
}

void LidarLITE::getDistance() {
	unsigned char distance[LidarLITE::READ_2_REGISTERS];
	unsigned char distanceRegister_1st[LidarLITE::WRITE_1_REGISTER];
	distanceRegister_1st[LidarLITE::WRITE_1_REGISTER-1] = LidarLITE::DISTANCE_1_2;

	//printf("Time =  %f starting Lidar::AquireDistance\n", m_timer->Get());

	do{Wait(.0001);} while (Busy());

	//printf("Time =  %f acquiring distance\n", m_timer->Get());

	/***********acquire distance**********/		//	WriteBulk() also works
	if ( lidarptr->Write(LidarLITE::COMMAND, LidarLITE::ACQUIRE_DC_CORRECT) )printf ( "Write operation failed! line %d\n", __LINE__ ); // initiate distance acquisition with DC stabilization

	do{Wait(.0001);} while (Busy());

	//printf("Time =  %f reading distance\n", m_timer->Get());

	/**********read distance**********/     // Read() does not work
	if ( lidarptr->WriteBulk(distanceRegister_1st, LidarLITE::WRITE_1_REGISTER)) printf ( "WriteBulk distance failed! line %d\n", __LINE__ );
	else
	if ( lidarptr->ReadOnly(LidarLITE::READ_2_REGISTERS, distance)) printf ( "ReadOnly distance failed! line %d\n", __LINE__ );

	unsigned int dist = (unsigned int)(distance[0]<<8) + (unsigned int)(distance[1]);

	printf("Distance= %d (0x%0x)\n", dist, dist);
}

LidarLITE::~LidarLITE() {
	// TODO Auto-generated destructor stub
}

