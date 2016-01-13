#ifndef ShooterSub_H
#define ShooterSub_H

#include "Commands/Subsystem.h"
#include "WPILib.h"

class ShooterSub: public Subsystem
{
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	ShooterSub();
	void InitDefaultCommand();
	void Shoot(float speed);
	std::shared_ptr<CANTalon> talonLeft;
	std::shared_ptr<CANTalon> talonRight;
};

#endif
