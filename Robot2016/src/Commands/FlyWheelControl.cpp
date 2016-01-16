#include "FlyWheelControl.h"

FlyWheelControl::FlyWheelControl()
{
	printf("FWC Constructor!\n");
	Requires(Robot::shooterSub.get());
	buttonAPressed = false;
}

// Called just before this Command runs the first time
void FlyWheelControl::Initialize()
{
	buttonAPressed = false;
	printf("FWC Init!\n");
}

// Called repeatedly when this Command is scheduled to run
void FlyWheelControl::Execute()
{
	//printf("FWC Execute!\n");
	buttonAPressed = Robot::oi->getGunnerJoystick()->GetRawButton(1); //change to trigger for variable speed?

	if(buttonAPressed)
	{
		Robot::shooterSub->Shoot(1);
	}
	else
	{
		Robot::shooterSub->Shoot(0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool FlyWheelControl::IsFinished()
{
	return false;
}

// Called once after isFinished returns true
void FlyWheelControl::End()
{

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FlyWheelControl::Interrupted()
{

}
