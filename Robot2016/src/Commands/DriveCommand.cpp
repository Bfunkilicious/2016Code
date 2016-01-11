// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "DriveCommand.h"

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

DriveCommand::DriveCommand(): Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	Requires(Robot::driveBaseSub.get());
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	XAxis = 0;
	YAxis = 0;
	RotAxis = 0;
	SetSpeed = 1;
	SetAngle = 0;
	ButtonAPressed = false;
	ButtonBPressed = false;
	ButtonXPressed = false;
	ButtonYPressed = false;
	FinalAutoRot = 0;
	AdjustedYaw = 0;
	isRotDone = true;
}
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void DriveCommand::Initialize() {
	XAxis = 0;
	YAxis = 0;
	RotAxis = 0;
	isRotDone = true;
}

// Called repeatedly when this Command is scheduled to run
void DriveCommand::Execute() {
	SmartDashboard::PutNumber("imuYaw", Robot::driveBaseSub->getAdjYaw());
	isRotDone = Robot::driveBaseSub->getIsRotDone();
	//printf("isRotDone%d\n", isRotDone);
	//Gets joystick values
	XAxis = Robot::oi->getDriveJoystick()->GetRawAxis(0);
	YAxis = Robot::oi->getDriveJoystick()->GetRawAxis(1);
	RotAxis = Robot::oi->getDriveJoystick()->GetRawAxis(4);

	//set angle to rotate to when button is pressed
	ButtonYPressed = Robot::oi->getDriveJoystick()->GetRawButton(4);
	if(ButtonYPressed == true)
	{

		SetAngle = 0;
	}

	ButtonAPressed = Robot::oi->getDriveJoystick()->GetRawButton(1);
	if(ButtonAPressed == true)
	{
		SetAngle = 180;
	}

	ButtonBPressed = Robot::oi->getDriveJoystick()->GetRawButton(2);
	if(ButtonBPressed == true)
	{
		SetAngle = 90;
	}

	ButtonXPressed = Robot::oi->getDriveJoystick()->GetRawButton(3);
	if(ButtonXPressed == true)
	{
		SetAngle = -90;
	}

	//Start if the button is pressed and we arent already going, or if we aren't done from previous conditions
	if(((ButtonYPressed == true||ButtonXPressed == true || ButtonAPressed == true || ButtonBPressed == true) && isRotDone == true) || (isRotDone == false))
	{
		//Rotate to Set Angle at Max Speed (1)
		FinalAutoRot = Robot::driveBaseSub->CalculateRotValue(SetAngle, SetSpeed);
	}
	printf("\n");
	printf("FinalAutoRot: %f\n", FinalAutoRot);
	printf("Angle: %f\n,", AdjustedYaw);
	printf("isRotDone: %d\n", Robot::driveBaseSub->getIsRotDone());
	if(Robot::driveBaseSub->getIsRotDoneOverride())
	{
		FinalAutoRot = 0;
	}
	SmartDashboard::PutNumber("XAxis", XAxis);
	SmartDashboard::PutNumber("YAxis", YAxis);
	SmartDashboard::PutNumber("RotAxis", RotAxis);

	//Sets up deadbands (this gets rid of the robot moving very slowly
	//even when you are not touching the joystick. Because the joystick
	//is never really at zero, (for example 0.0389583), this checks if
	//any joystick is in the range of -0.20 to 0.20 and if it is sets it
	//to zero, if not, sends the current value to the MechDrive function.

	//Also squares the values which increases sensitivity, and (mostly) blends out
	//the discontinuity you would have with just the dead band (there's no step from 0 to .2,
	//the lowest value is 0 to .04 now.

	if(XAxis > -0.20 && XAxis < 0.20)
	{
		XAxis = 0;
	}
	else
	{
		XAxis = XAxis * fabs(XAxis);
	}

	if(YAxis > -0.20 && YAxis < 0.20)
	{
		YAxis = 0;
	}
	else
	{
		YAxis = YAxis * fabs(YAxis);
	}

	if(RotAxis > -0.20 && RotAxis < 0.20)
	{
		RotAxis = 0;
	}
	else
	{
		RotAxis = RotAxis * fabs(RotAxis);
	}

	AdjustedYaw = Robot::driveBaseSub->getAdjYaw();
	if(RotAxis == 0)
	{
		//printf("Noone is moving the rotate joystick\n");
		Robot::driveBaseSub->setIsRotDoneOverride(false);
		//printf("FinalAutoRot%f\n", FinalAutoRot);
		Robot::driveBaseSub->MecanumDrive(XAxis,YAxis,FinalAutoRot,AdjustedYaw);
	}
	else
	{
		//printf("Someone is moving the rotate joystick\n");
		Robot::driveBaseSub->setIsRotDoneOverride(true);
		Robot::driveBaseSub->setIsRotDone(true);
		Robot::driveBaseSub->setTimesThroughLoop(0);
		Robot::driveBaseSub->MecanumDrive(XAxis,YAxis,RotAxis,AdjustedYaw);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveCommand::IsFinished() {
    return false;
}

// Called once after isFinished returns true
void DriveCommand::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveCommand::Interrupted() {

}
