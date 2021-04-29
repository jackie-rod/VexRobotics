#pragma config(Sensor, dgtl1,  LeftEncoder,    sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  RightEncoder,   sensorQuadEncoder)
#pragma config(Motor,  port1,           LeftBase,      tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           RightBase,     tmotorVex393_MC29, openLoop)
#pragma platform(VEX2)

#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

////////////////////////////////VOID/SETUP////////////////////////////////////////////
////////////////////////////////VOID/SETUP////////////////////////////////////////////
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

int degreesToTicks (float degree)
{
	int ticksPerTurn = 3000; // ? not sure, find your own constant!
	int ticks = degree*ticksPerTurn/360;
	return ticks;
}

int degreesToTicksBeta (float degree) // included this here anyway, in case you need it.
{
	float turningRadius = 6.5;
	float turningCircumference = 2*PI*turningRadius;
	int ticksPerTurn = inchToTicks(turningCircumference);
	int ticks = degree*ticksPerTurn/360;
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

void moveRightBase (int speed)
{
	motor[RightBase] = speed;
}

void moveLeftBase (int speed)
{
	motor[LeftBase] = speed;
}

void turnBase (int speed) // positive is clockwise
{
	motor[LeftBase] = speed;
	motor[RightBase] = -speed;
}

void resetEn()
{
	SensorValue[LeftEncoder] = 0;
	SensorValue[RightEncoder] = 0;
}

void PIDbaseControl(float target, float waitTime, float maxPower = 1)
{
	////////////////////////////SETTING/FLOATS/AND/INTEGERS///////////////////////////
	////////////////////////////SETTING/FLOATS/AND/INTEGERS///////////////////////////
	int error;
	int proportion;
	float Kp = 0.2;
	/*P: Proportional
	The Proportional component provides the bulk of the power for controlling your
	system. The key objective is to give a large amount of power when there is a long
	way to go, but only a small amount of power when youre nearly at your setpoint.
	This results in a smooth deceleration through the movement as you approach the
	setpoint.*/



	int integralRaw;
	float integral;
	float Ki = 0.05;
	/*I: Integral
	with the proportional component that once the
	error becomes small you have very little power, and might see some significant
	remaining error that just isn’t eliminated – the integral will get rid of this for you
	by slowly increasing the speed. The integral is going to be concerned with looking
	back in time over all the errors your system has calculated.*/


	int lastError;
	int derivative;
	float Kd = 0.5;
	/*D: Derivative
	The idea of the derivative is to look at the rate of change of our error. bit? The contribution from the derivative
	will be in the direction opposite to your current direction of travel, with a larger
	magnitude for a greater speed. It typically will be outweighed by the proportional
	and integral components, but if there’s some deviation from “normal” and the
	robot is going faster for some reason, the derivative component will become
	larger and your output power will be reduced.
	CALCULUS DEFINITION : slope of the given function on a graph. aka how fast it is going --> velocity = distance/time
	THE DERIVATIVE WILL ALWAYS BE NEGATIVE*/

	int finalPower;
	float integralPowerLimit = 50/Ki;
	//area that proportion is not specific enough therefore the integral is activated
	float integralActiveZone = inchToTicks(3);
	bool timerBool = true;

	float Kp_C = 0.01;
	int error_drift;
	float proportion_drift;

	resetEn();
	clearTimer(T1);

	//////////////////////////////ALGORITHM/START///////////////////////////////////////
	//////////////////////////////ALGORITHM/START///////////////////////////////////////
	while(time1[T1] < fixTimerValue(waitTime))
	{
		////////////////////////////PROPORTION/CALCULATION////////////////////////////////
		////////////////////////////PROPORTION/CALCULATION////////////////////////////////
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

		////////////////////////////INTEGRAL/CALCULATION//////////////////////////////////
		////////////////////////////INTEGRAL/CALCULATION//////////////////////////////////
		//accumulative number starts when error is in inegral active zone
		if(abs(error) < integralActiveZone && error != 0)
			integralRaw = integralRaw + error;
		else
			integralRaw = 0;

		////////////////////////////INTEGRAL/POWER/LIMIT//////////////////////////////////
		////////////////////////////INTEGRAL/POWER/LIMIT//////////////////////////////////
		if(integralRaw > integralPowerLimit)
			integralRaw = integralPowerLimit;
    if(integralRaw < -integralPowerLimit)
    	integralRaw = -integralPowerLimit;

		integral = Ki*integralRaw;

		////////////////////////////DERIVATIVE/CALCULATION////////////////////////////////
		////////////////////////////DERIVATIVE/CALCULATION////////////////////////////////
		derivative = Kd*(error - lastError);
		lastError = error;

		if(error == 0)
			derivative = 0;

		finalPower = proportion + integral + derivative;

		////////////////////////////POWER/PERCENTAGE/CONVERSION///////////////////////////
		////////////////////////////POWER/PERCENTAGE/CONVERSION///////////////////////////
		if (finalPower > maxPower*127)
			finalPower = maxPower*127;
		else if (finalPower < -maxPower*127)
			finalPower = -maxPower*127;

		////////////////////////////DRIFT/ERROR/CORRECTION////////////////////////////////
		////////////////////////////DRIFT/ERROR/CORRECTION////////////////////////////////
		error_drift = Kp_C*(SensorValue[RightBase]-SensorValue[LeftBase]); //drift correction code
		//if robot is tilting left drift error is negative
		//if robot is tilting right drift error is positive
		if(error_drift > 30)
		{
		error_drift = 30;
		}
		if(error_drift < -30)
		{
		error_drift = -30;
		}

		finalPowerLeft = finalPower - driftError;
		finalPowerRight = finalPower + driftError;
		moveBase(finalPowerLeft, finalPowerRight);
		delay(40);

		////////////////////////////EXIT/LOOP/////////////////////////////////////////////
		////////////////////////////EXIT/LOOP/////////////////////////////////////////////
		//gets out of while loop when error reaches 30
		if (error < 30)
			timerBool = false;
		if (timerBool)
			clearTimer(T1);
	}
	moveBase(0);
}

void PIDbaseTurn (int target, float waitTime, float maxPower = 1)
{
	float Kp = 0.2;
	float Ki = 0.05;
	float Kd = 0.5;
	int error;
	float proportion;
	int integralRaw;
	float integral;
	int lastError;
	int derivative;
	float integralActiveZone = inchToTicks(3);
	float integralPowerLimit = 50/Ki;
	int finalPower;
	bool timerBool = true;
	resetEn();
	clearTimer(T1);

	//////////////////////////////ALGORITHM/START///////////////////////////////////////
	//////////////////////////////ALGORITHM/START///////////////////////////////////////
	while (time1[T1] < fixTimerValue(waitTime))
	{
		////////////////////////////PROPORTION/CALCULATION////////////////////////////////
		////////////////////////////PROPORTION/CALCULATION////////////////////////////////
		error = degreesToTicks(target)-(SensorValue[LeftBase]-SensorValue[RightBase]);
		proportion = Kp*error;

		////////////////////////////INTEGRAL/CALCULATION//////////////////////////////////
		////////////////////////////INTEGRAL/CALCULATION//////////////////////////////////
		if (abs(error) < integralActiveZone && error != 0)
			integralRaw = integralRaw+error;
		else
			integralRaw = 0;

		////////////////////////////INTEGRAL/POWER/LIMIT//////////////////////////////////
		////////////////////////////INTEGRAL/POWER/LIMIT//////////////////////////////////
		if (integralRaw > integralPowerLimit)
			integralRaw = integralPowerLimit;
		if (integralRaw < -integralPowerLimit)
			integralRaw = -integralPowerLimit;

		integral = Ki*integralRaw;

		////////////////////////////DERIVATIVE/CALCULATION////////////////////////////////
		////////////////////////////DERIVATIVE/CALCULATION////////////////////////////////
		derivative = Kd*(error-lastError);
		lastError = error;

		if (error == 0)
			derivative = 0;

		finalPower = proportion+integral+derivative; //proportion+derivative+integral

		////////////////////////////POWER/PERCENTAGE/CONVERSION///////////////////////////
		////////////////////////////POWER/PERCENTAGE/CONVERSION///////////////////////////
		if (finalPower > maxPower*127)
			finalPower = maxPower*127;
		else if (finalPower < -maxPower*127)
			finalPower = -maxPower*127;
		//final method
		turnBase(finalPower);
		wait1Msec(40);

		////////////////////////////EXIT/LOOP/////////////////////////////////////////////
		////////////////////////////EXIT/LOOP/////////////////////////////////////////////
		if (error < 30)
			timerBool = false;
		if (timerBool)
			clearTimer(T1);
	}
	turnBase(0);
}

task autonomous()
{
	PIDbaseControl(12,0,0.7); // move forward 12 inches with 0.25 sec delay;
	PIDbaseTurn(1000,1,); // turn right 1000 counts with 1 sec delay;
	PIDbaseControl(-7.5,1); // back off 7.5 inches with 1 sec delay;
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
