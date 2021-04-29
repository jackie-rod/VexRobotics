#pragma config(Motor,  port1,           WHEEL1,        tmotorVex393TurboSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           WHEEL3,        tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port3,           LIFT1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           CLAW1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           CLAW2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           LAUNCH,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           LIFT2,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           WHEEL2,        tmotorVex393TurboSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          WHEEL4,        tmotorVex393TurboSpeed_HBridge, openLoop, reversed)
#pragma platform(VEX2)
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(120)
#include "Vex_Competition_Includes.c"
void pre_auton()
{
}
task autonomous()
{
}
task usercontrol()
{
	 							//-----------------------------STRAFING_AND_DEADZONE------------------------------//

	  //Create "deadzone" variables. Adjust threshold value to increase/decrease deadzone
  int X2 = 0, Y1 = 0, X1 = 0, threshold = 15;

  //Loop Forever
  while(1 == 1)
  {
    //Create "deadzone" for Y1/Ch3
    if(abs(vexRT[Ch3]) > threshold)
      Y1 = vexRT[Ch3];
    else
      Y1 = 0;
    //Create "deadzone" for X1/Ch4
    if(abs(vexRT[Ch4]) > threshold)
      X1 = vexRT[Ch4];
    else
      X1 = 0;
    //Create "deadzone" for X2/Ch1
    if(abs(vexRT[Ch1]) > threshold)
      X2 = vexRT[Ch1];
    else
      X2 = 0;

    //Remote Control Commands
    motor[port1] = Y1 - X2 - X1;
    motor[port2] =  Y1 - X2 + X1;
    motor[port9] = Y1 + X2 + X1;
    motor[port10] =  Y1 + X2 - X1;

}
