#include "SetOffset.h"

SetOffset::SetOffset(float offset)
{
	printf("SetOffset Constructor!\n");
	Requires(Robot::driveBaseSub.get());
	isDone = false;
	offsetInput = offset;
}

// Called just before this Command runs the first time
void SetOffset::Initialize()
{
	isDone = false;
}

// Called repeatedly when this Command is scheduled to run
void SetOffset::Execute()
{
	Robot::driveBaseSub->setAdjYaw(offsetInput);
	isDone = true;
}

// Make this return true when this Command no longer needs to run execute()
bool SetOffset::IsFinished()
{
	return isDone;
}

// Called once after isFinished returns true
void SetOffset::End()
{

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetOffset::Interrupted()
{

}
