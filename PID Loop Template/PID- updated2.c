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
	int error;
	int proportion;
	float Kp = 0.2;
	/*P: Proportional
	The Proportional component provides the bulk of the power for controlling your
	system. The key objective is to give a large amount of power when there is a long
	way to go, but only a small amount of power when youre nearly at your setpoint.
	This results in a smooth deceleration through the movement as you approach the
	setpoint.*/

	float integralActiveZone = inchToTicks(3);
	//area that proportion is not specific enough therefore the integral is activated
	int integralRaw;
	float integralPowerLimit = 50/Ki;
	float integral;
	float Ki = 0.05;
	/*I: Integral
	with the proportional component that once the
	error becomes small you have very little power, and might see some significant
	remaining error that just isn’t eliminated – the integral will get rid of this for you
	by slowly increasing the speed. The integral is going to be concerned with looking
	back in time over all the errors your system has calculated.*/

	int finalPower;
	bool timerBool = true;

	resetEn();
	clearTimer(T1);



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

		//accumulative number starts when error is in inegral active zone
		if(abs(error) < integralActiveZone && error != 0)
			integralRaw = integralRaw + error;
		else
			integralRaw = 0;

		if(integralRaw > integralPowerLimit)
			integralRaw = integralPowerLimit;
    if(integralRaw < -integralPowerLimit)
    	integralRaw = -integralPowerLimit;

		integral = Ki*integralRaw;

		finalPower = proportion + integral;
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
{
PIDbaseControl(5, 3);
}


task usercontrol()
{
  // User control code here, inside the loop

  while (true)
  {
  	//armControl(armMotorPort, upButton, downButton, armSpeed);
    tankControl(Ch2, Ch3, 2);
  }
}
