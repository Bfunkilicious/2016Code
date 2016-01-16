#include "TigerDrive.h"

TigerDrive::TigerDrive(AHRS* imuP)
{
	printf("constructing TigerDrive!\n");
	imuPointer = imuP;
	imuYaw = 0;
	angleTolerance = 2; //change if you want.
	rotateLoopCheck = 5;
	originalAngle = 0;
	speedWhileRotating = 0;
	speedScaleFactor = 0.5;
	imuRaw = 0;
	imuScaled = 0;
	scaledAngle = 0;
	spinDirection = 0;
	calculatedRotate = 0;
	tooFarCW = false;
	tooFarCCW = false;
	degreesToAngle = 0;
	degreesToAngleAbs = 0;
	timesThroughLoop = 0;
	isRotDone = true;
	yawOffset = 0;
	calculatedOffset = 0;
	isRotDoneOverride = false;
}

float TigerDrive::CalculateRotValue(float angle, float speed)
{
	//printf("I am in CalculateRotValue\n");
	originalAngle = angle;
	speedScaleFactor = speed;
	//printf("originalAngle%f\n", originalAngle);
	//printf("speedScaleFactor%f\n", speedScaleFactor);
	tooFarCW = false;
	tooFarCCW = false;
	calculatedRotate = 0;
	imuYaw = getAdjYaw();
	imuScaled = imuYaw + 1000;
	scaledAngle = originalAngle + 1000;
	//sets spin direction and degrees to rotate to
	//printf("before\n");
	if(imuScaled > (scaledAngle + angleTolerance))
	{
		tooFarCW = true;
		spinDirection = -1;
		degreesToAngle = imuScaled - scaledAngle;
		//printf("tooFarCW is true\n");
	}
	//sets spin direction and degrees to rotate to
	if(imuScaled < (scaledAngle - angleTolerance))
	{
		tooFarCCW = true;
		spinDirection = 1;
		degreesToAngle = scaledAngle - imuScaled;
		//printf("tooFarCCW is true\n");
	}
	//printf("after\n");
	//printf("imuScaled: %f3.2 scaledAngle: %f3.2 tolerance: %f3.2 degreesToAngle: %f3.2 \n", imuScaled, scaledAngle, angleTolerance, degreesToAngle);
	//makes sure when rotating to 180 robot does not turn wrong way
	if(degreesToAngle > 180)
	{
		degreesToAngle = 360 - degreesToAngle;
		spinDirection = spinDirection * -1;
	}

	//printf("tooFarCW: %d\n", tooFarCW);
	//printf("tooFarCCW: %d\n", tooFarCCW);
	//only start spinning if we need to
	if(tooFarCW || tooFarCCW)
	{
		isRotDone = false;
		degreesToAngleAbs = fabs(degreesToAngle);
		//why not an exponential function? :(
		if(degreesToAngleAbs <= 180)
		{
			speedWhileRotating = 1;
		}
		if(degreesToAngleAbs <= 90)
		{
			speedWhileRotating = 0.4;
		}
		if(degreesToAngleAbs <= 60)
		{
			speedWhileRotating = 0.3;
		}
		if(degreesToAngleAbs <= 30)
		{
			speedWhileRotating = 0.2;
		}
		calculatedRotate = spinDirection * speedWhileRotating;
		calculatedRotate = calculatedRotate * speedScaleFactor;
		timesThroughLoop = 1;
		//printf("TimesThroughLoopIf: %d\n", timesThroughLoop);
	}
	else
	{
		//printf("rotateLoopCheck = %d\n", rotateLoopCheck);
		//printf("TimesThroughLoopElse: %d\n", timesThroughLoop);
		//printf("We do not need to rotate anymore at least i think");
		if(timesThroughLoop == rotateLoopCheck || timesThroughLoop == 0)
		{
			//printf("times through loop == loopcheck or timesthroughloop == 0");
			isRotDone = true;
			timesThroughLoop = 0;
			calculatedRotate = 0;
		}
		timesThroughLoop = timesThroughLoop + 1;
		//printf("TimesThroughLoop%d\n", timesThroughLoop);
	}

	//printf("isRotDoneOverride: %d\n", isRotDoneOverride);
	if(isRotDoneOverride)
	{
		calculatedRotate = 0;
	}
	return calculatedRotate;
}


float TigerDrive::getAdjYaw()
{
	imuRaw = imuPointer->GetYaw();
	calculatedOffset = imuRaw + yawOffset;
	if(calculatedOffset >= 180)
	{
		calculatedOffset = calculatedOffset - 360;
	}
	return calculatedOffset;
}

void TigerDrive::setAdjYaw(float offset)
{
	yawOffset = offset;
}

void TigerDrive::setIsRotDone(bool isDone)
{
	isRotDone = isDone;
}

void TigerDrive::setIsRotDoneOverride(bool isDone)
{
	isRotDoneOverride = isDone;
}

void TigerDrive::setTimesThroughLoop(int timeLoop)
{
	timesThroughLoop = timeLoop;
}

float TigerDrive::getImuYaw()
{
	return imuYaw;
}

bool TigerDrive::getisRotDone()
{
	return isRotDone;
}

bool TigerDrive::getisRotDoneOverride()
{
	return isRotDoneOverride;
}
