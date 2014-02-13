/*
 * help.c
 *
 *  Created on: Oct 28, 2013
 *      Author: Sarang
 */

#include "main.h"


//extern Gyro gyro;

// Analog inputs
#define ArmPotentiometer 13
#define GyroPort 14
// Digital ports
#define RightMorpheus 11
#define LeftMorpheus 12
// Motors
#define LeftBackMotor1 2
#define LeftBackMotor2 3
#define RightBackMotor1 4
#define RightBackMotor2 5
#define LeftFrontMotor1 6
#define RightFrontMotor1 7
#define Intake 8
#define Arm1 9
#define Arm2 10

#define TURN_SPEED 2


int cubicMap(int x) {
	return ((((x * 3) / 25) * ((x * 3) / 25) * ((x * 3) / 25) / 27 + x / 2) * 2)
			/ 3;
}
