// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include "RobotMap.h"


// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION
std::shared_ptr<CANTalon> RobotMap::driveBaseSubFrontLeftTalon;
std::shared_ptr<CANTalon> RobotMap::driveBaseSubFrontRightTalon;
std::shared_ptr<CANTalon> RobotMap::driveBaseSubBackLeftTalon;
std::shared_ptr<CANTalon> RobotMap::driveBaseSubBackRightTalon;
std::shared_ptr<CANTalon> RobotMap::shooterSubLeftTalon;
std::shared_ptr<CANTalon> RobotMap::shooterSubRightTalon;
std::shared_ptr<RobotDrive> RobotMap::driveBaseSubRobotDriveController;
// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=ALLOCATION

void RobotMap::init() {
	printf("RobotMap init! \n");
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

    driveBaseSubFrontLeftTalon.reset(new CANTalon(8));
    
    driveBaseSubFrontRightTalon.reset(new CANTalon(6));
    
    driveBaseSubBackLeftTalon.reset(new CANTalon(9));
    
    driveBaseSubBackRightTalon.reset(new CANTalon(2));
    
    shooterSubLeftTalon.reset(new CANTalon(1));

    shooterSubRightTalon.reset(new CANTalon(3));

    driveBaseSubRobotDriveController.reset(new RobotDrive(driveBaseSubFrontLeftTalon, driveBaseSubBackLeftTalon,
              driveBaseSubFrontRightTalon, driveBaseSubBackRightTalon));
    
    driveBaseSubRobotDriveController->SetSafetyEnabled(false);
        driveBaseSubRobotDriveController->SetExpiration(0.1);
        driveBaseSubRobotDriveController->SetSensitivity(0.5);
        driveBaseSubRobotDriveController->SetMaxOutput(1.0);

        driveBaseSubRobotDriveController->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
        driveBaseSubRobotDriveController->SetInvertedMotor(RobotDrive::kRearRightMotor, true);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
   printf("end of robotMap!\n");
}
