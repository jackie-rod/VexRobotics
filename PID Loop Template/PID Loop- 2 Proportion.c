#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  LEFTen,         sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  RIGHTen,        sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           LEFT,          tmotorVex393_HBridge, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port2,           RIGHT,         tmotorVex393_MC29, openLoop, encoderPort, I2C_2)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//


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

int inchToTicks(float target)
{
	int ticks;
	ticks = inch * 99.82198;
	return ticks;
}

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

void PIDforward(int target,)
{

	float Kp = 0.2;
	int error;
	int proportion;
	int finalPower;
	resetEn(40);

	while(true)
	{
		//__________________________PROPORTION__________________________________//
		// if you only have proportion robot never stops and keeps going infinitely because loop is always true
		error = inchToTicks (target) - (nMotorEncoder[LEFTen] + nMotorEncoder[RIGHTen]);

		proportion = Kp * error

		finalPower = proportion		//proportion+integral+derivative

		forward(finalPower);
		wait1Msec(40);
	}
}


//__________________________AUTONOMOUS______________________________________//
task autonomous()
{
		PIDforward(12); //forward 12 inches
}
//__________________________________________________________________________//
//__________________________________________________________________________//