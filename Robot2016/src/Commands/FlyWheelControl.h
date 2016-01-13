#ifndef FlyWheelControl_H
#define FlyWheelControl_H

#include "../Robot.h"
#include "WPILib.h"

class FlyWheelControl: public Command
{
public:
	FlyWheelControl();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
	bool buttonAPressed;
};

#endif
