#include "DriveBaseSub.h"
#include "../RobotMap.h"
#include "../Commands/DriveCommand.h"

DriveBaseSub::DriveBaseSub() :
		Subsystem("DriveBaseSub")
{
	printf("driveBaseSub constructor!\n");
	talonFL = RobotMap::driveBaseSubFrontLeftTalon;
	talonFR = RobotMap::driveBaseSubFrontRightTalon;
	talonBL = RobotMap::driveBaseSubBackLeftTalon;
	talonBR = RobotMap::driveBaseSubBackRightTalon;
	robotDrive = RobotMap::driveBaseSubRobotDriveController;
	imuRobot = new AHRS(SPI::Port::kMXP);
	tigerDrive = new TigerDrive(imuRobot);
	IMU_Yaw = 0;
	calculatedoffset = 0;
	yawoffset = 0;
	printf("Out of driveBaseSub!\n");
}

void DriveBaseSub::InitDefaultCommand()
{
	printf("setting default command for drive!\n");
	// Set the default command for a subsystem here.
	SetDefaultCommand(new DriveCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void DriveBaseSub::MecanumDrive(float x, float y, float rot, float gyro)
{
	robotDrive->MecanumDrive_Cartesian(x,y,rot,gyro);
}

float DriveBaseSub::CalculateRotValue(float setAngle, float setSpeed)
{
	return tigerDrive->CalculateRotValue(setAngle, setSpeed);
}

float DriveBaseSub::getAdjYaw()
{
	return tigerDrive->getAdjYaw();
}

void DriveBaseSub::setAdjYaw(float offset)
{
	tigerDrive->setAdjYaw(offset);
}

void DriveBaseSub::setIsRotDone(bool isDone)
{
	tigerDrive->setIsRotDone(isDone);
}

void DriveBaseSub::setIsRotDoneOverride(bool isDone)
{
	tigerDrive->setIsRotDoneOverride(isDone);
}

void DriveBaseSub::setTimesThroughLoop(int timeLoop)
{
	tigerDrive->setTimesThroughLoop(timeLoop);
}

float DriveBaseSub::getImuYaw()
{
	return tigerDrive->getImuYaw();
}

bool DriveBaseSub::getIsRotDone()
{
	return tigerDrive->getisRotDone();
}

bool DriveBaseSub::getIsRotDoneOverride()
{
	return tigerDrive->getisRotDoneOverride();
}
