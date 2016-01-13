#include "DriveBaseSub.h"
#include "../RobotMap.h"
#include "../Commands/DriveCommand.h"

const int UPDATE_RATE_HZ = 50;
const int BAUD_RATE = 57600;

DriveBaseSub::DriveBaseSub() :
		Subsystem("DriveBaseSub")
{
	talonFL = RobotMap::driveBaseSubFrontLeftTalon;
	talonFR = RobotMap::driveBaseSubFrontRightTalon;
	talonBL = RobotMap::driveBaseSubBackLeftTalon;
	talonBR = RobotMap::driveBaseSubBackRightTalon;
	robotDrive = RobotMap::driveBaseSubRobotDriveController;
	//imuSerialPort = new SerialPort(57600, SerialPort::Port::kUSB);
	imuRobot = new AHRS(SerialPort::Port::kUSB);
	tigerDrive = new TigerDrive(imuRobot);
	IMU_Yaw = 0;
	calculatedoffset = 0;
	yawoffset = 0;
	printf("driveBaseSub constructor!\n");
}

void DriveBaseSub::InitDefaultCommand()
{
	// Set the default command for a subsystem here.
	SetDefaultCommand(new DriveCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void DriveBaseSub::MecanumDrive(float x, float y, float rot, float gyro)
{
	robotDrive->MecanumDrive_Cartesian(x,y,rot,gyro);
}

void DriveBaseSub::TractionDrive(float y, float rot, bool squared)
{
	robotDrive->ArcadeDrive(y,rot,squared);
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
