#include "ShooterSub.h"
#include "../RobotMap.h"
#include "../Commands/FlyWheelControl.h"

ShooterSub::ShooterSub() :
		Subsystem("ShooterSub")
{
	printf("constructing shooter sub!\n");
	talonLeft = RobotMap::shooterSubLeftTalon;
	talonRight = RobotMap::shooterSubRightTalon;
}

void ShooterSub::InitDefaultCommand()
{
	printf("setting default command for shooter sub!\n");
	// Set the default command for a subsystem here.
	SetDefaultCommand(new FlyWheelControl());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void ShooterSub::Shoot(float speed)
{
	Robot::shooterSub->talonLeft->Set(speed);
	Robot::shooterSub->talonRight->Set(speed);
}
