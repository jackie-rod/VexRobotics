#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  LEFTen,         sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  RIGHTen,        sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           LEFT,          tmotorVex393_HBridge, PIDControl, encoderPort, I2C_1)
#pragma config(Motor,  port2,           RIGHT,         tmotorVex393_MC29, PIDControl, encoderPort, I2C_2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

//___________________________TICKS/INCH CONVERSION___________________________//
//reading: ticks -- 627.2tick/revolution (high torque)
//1 turn of weels -- 4 in. wheels (omniwheels) = 627.2 *2ticks (because 2 motor encoders)
//1 turn of each wheel --4 * pi in.
//4*pi inches = 627.2 *2ticks
//how many ticks/in?
//ticks/in = 627.2 * 2/ 4 * pi = 99.82198
//this means: for every 1 inch the robot moves with an omniwheel, with high torque is (approximately 100)
//__________________________________________________________________________//
//__________________________________________________________________________//

void pre_auton()
{
  bStopTasksBetweenModes = true;
}

int inchToTicks(float inch)
{
	int ticks;
	ticks = inch * 99.82198;
	return ticks;
}

int betterTime(float rawSeconds) // converts 1000 miliseconds to 1 second
{
	int miliseconds; 							 //allows you to put 1 second in parameter -- (127,1) -- 127 speed at 1 second
	miliseconds = rawSeconds * 1000;
		if (miliseconds < 250) 				 //robot always stops after every command for at least .25 seconds
		{
			miliseconds = 250;
		}//if
		return miliseconds;
}//int

void resetEn(int mSec)
{
	nMotorEncoder[LEFTen] = 0;
	nMotorEncoder[RIGHTen] = 0;
}

void forward(int speed)
{
	motor[LEFT] = speed;
	motor[RIGHT] = speed;
}

void PIDforward(float target, float waitTime, float maxPower = 1)
{

	float Kp = 0.2;  //constant proportion
	float Ki = 0.04; //constant integral
	float Kd = 0.5;  //constant deivative

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

	resetEn(40);
	clearTimer(T1);


	while(time1(T1) < betterTime(waitTime))
	{
		error = inchToTicks (target) - (nMotorEncoder[LEFTen] + nMotorEncoder[RIGHTen]);
		proportion = Kp * error;

	//INTEGRAL accumulative error -- exponentially grows
		if (abs(error) < integralActiveZone && error != 0)
		{
				integralRaw = integralRaw + error;
		}

		else //reset integral command
		{
				integralRaw = 0;
		}

		if (integralRaw > integralPowerLimit)
		{
				integralRaw = integralPowerLimit;
		}

		if(integralRaw < -integralPowerLimit)
		{
			integralRaw = -integralPowerLimit;
		}

		integral = Ki * integralRaw;
		derivative = Kd * (error - lastError);
		lastError = error;

		if(error == 0)
		{
			derivative = 0;
		}

	//FINAL
		finalPower = proportion + integral + derivative;
			//HARD LIMITER
			if (finalPower > maxPower * 127)
			{
				finalPower = maxPower * 127;
			}
			else if (finalPower < -maxPower * 127)
			{
				finalPower = -maxPower * 127;
			}

		forward(finalPower);
		wait1Msec(40);

			if (error < 30)
			{
				timerBool = false;
			}//if

			if (timerBool)
			{
				clearTimer(T1);
			}//if

	}//while
}//void


//__________________________AUTONOMOUS______________________________________//
task autonomous()
{
		PIDforward(12,0,0.7); //forward 12 inches, 250 delay, .7 percent power
}
//__________________________________________________________________________//
//__________________________________________________________________________//

task usercontrol()
{
	 UserControlCodePlaceholderForTesting();
}
