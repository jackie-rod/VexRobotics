#pragma config(Sensor, dgtl1,  LeftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightEncoder,   sensorQuadEncoder)
#pragma config(Motor,  port1,           LeftBase,      tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           RightBase,     tmotorVex393_MC29, openLoop)
#pragma platform(VEX2)

#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

void pre_auton()
{}

int inchToTicks(float inch)
{
	/*Distance traveled = Wheel rotations * circumference =>
	Distance traveled = (Degrees turned / 360) * circumference =>
	Distance traveled = (Encoder ticks / 360) * circumference =>
	Encoder ticks = (360 / circumference) * Distance to travel*/
	int ticks;
	ticks = inch * 99.82198;
	return ticks;
}

int fixTimerValue(float rawSeconds)
{
	int miliseconds;
	miliseconds = rawSeconds*1000;
	if (miliseconds < 250)
		miliseconds = 250;
	return miliseconds;
}

void moveBase(int speed)
{
	motor[LeftBase] = speed;
	motor[RightBase] = speed;
}

void resetEn()
{
	SensorValue[LeftEncoder] = 0;
	SensorValue[RightEncoder] = 0;
}

void PIDbaseControl(float target, float waitTime)
{
	//proportion calculates the error
	float Kp = 0.2;
	int error;
	int proportion;
	int finalPower;
	bool timerBool = true;
	resetEn();
	clearTimer(T1);

	/*P: Proportional
	The Proportional component provides the bulk of the power for controlling your
	system. The key objective is to give a large amount of power when there is a long
	way to go, but only a small amount of power when you’re nearly at your setpoint.
	This results in a smooth deceleration through the movement as you approach the
	setpoint.*/

	while(time1[T1] < fixTimerValue(waitTime))
	{
		/*To achieve the nice smooth deceleration that the Proportional Controller
		provides, we could simply set the power of our motors to be equal to the error, like
		so:*/
		error = inchToTicks(target) - (SensorValue[LeftEncoder] + SensorValue[RightEncoder]);

		/*However, you may find that the speed values don’t seem to be scaled right. The
		robot may be a bit too gentle approaching the target, and may in fact not have
		enough power at all to reach the setpoint when the error becomes small. Or
		alternatively, the robot might be a bit aggressive, and it might significantly
		overshoot, and then overcorrect, in a never-ending cycle.
		To combat this issue, we introduce another value, the proportional constant (kP).
		Simply put, we multiply the error by kP when we assign the error to the output
		power.*/
		proportion = Kp*error;
		finalPower = proportion;
		moveBase(finalPower);
		delay(40);

		//gets out of while loop when error reaches 30
		if (error < 30)
			timerBool = false;
		if (timerBool)
			clearTimer(T1);
	}
	moveBase(0);
}

task autonomous()
{}


task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {

  }
}
