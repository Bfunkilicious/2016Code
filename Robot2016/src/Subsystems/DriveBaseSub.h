#ifndef DriveBaseSub_H
#define DriveBaseSub_H

#include "Commands/Subsystem.h"
#include "WPILib.h"
#include "../TigerDrive/TigerDrive.h"

class DriveBaseSub: public Subsystem
{
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	std::shared_ptr<CANTalon> talonFL;
	std::shared_ptr<CANTalon> talonFR;
	std::shared_ptr<CANTalon> talonBL;
	std::shared_ptr<CANTalon> talonBR;
	std::shared_ptr<RobotDrive> robotDrive;
	//SerialPort *imuSerialPort;
	AHRS *imuRobot;
	TigerDrive *tigerDrive;
	DriveBaseSub();
	void InitDefaultCommand();
	void MecanumDrive(float x, float y, float rot, float gyro);
	void TractionDrive(float y, float rot, bool squared);
	float CalculateRotValue(float setAngle, float setSpeed);
	void IMU_YAWoffset(float offset);
	float getAdjYaw();
	void setAdjYaw(float offset);
	float getImuYaw();
	bool getIsRotDone();
	void setIsRotDone(bool isDone);
	void setIsRotDoneOverride(bool isDone);
	void setTimesThroughLoop(int timeLoop);
	bool getIsRotDoneOverride();
	float calculatedoffset; //calculated for imu offset
	float yawoffset; //param
	float IMU_Yaw; //return value for imuoffset
};

#endif
