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
	int integralRaw;
	float integral;
	float Ki = 0.05;
	int lastError;
	int derivative;
	float Kd = 0.5;
	int finalPower;
	float integralPowerLimit = 50/Ki;
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
		error = inchToTicks(target) - (SensorValue[LeftEncoder] + SensorValue[RightEncoder]);
		proportion = Kp*error;

		////////////////////////////INTEGRAL/CALCULATION//////////////////////////////////
		////////////////////////////INTEGRAL/CALCULATION//////////////////////////////////
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
		error_drift = SensorValue[RightBase]-SensorValue[LeftBase];
		proportion_drift = Kp_C*error_drift;

		moveLeftBase(finalPower+proportion_drift);
		moveRightBase(finalPower-proportion_drift);

		//final method
		moveBase(finalPower);
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
