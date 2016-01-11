#ifndef SetOffset_H
#define SetOffset_H

#include "Commands/Subsystem.h"
#include "../Robot.h"

class SetOffset: public Command
{
public:
	SetOffset(float offset);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	bool isDone;
	float offsetInput;
};

#endif
