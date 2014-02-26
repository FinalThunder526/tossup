#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    ArmPotentiometer, sensorPotentiometer)
#pragma config(Sensor, in2,    Gyroscope,      sensorGyro)
#pragma config(Sensor, in3,    AutonPotentiometer, sensorPotentiometer)
#pragma config(Sensor, dgtl2,  MorpheusL,      sensorDigitalOut)
#pragma config(Sensor, dgtl3,  MorpheusR,      sensorDigitalOut)
#pragma config(Sensor, dgtl4,  HangingL,       sensorDigitalOut)
#pragma config(Sensor, dgtl5,  HangingR,       sensorDigitalOut)
#pragma config(Sensor, dgtl6,  HangLock,       sensorDigitalOut)
#pragma config(Sensor, dgtl8,  Catapult,       sensorDigitalOut)
#pragma config(Sensor, dgtl9,  AutonButton,    sensorTouch)
#pragma config(Sensor, dgtl10, RedLED,         sensorLEDtoVCC)
#pragma config(Sensor, dgtl11, YellowLED,      sensorLEDtoVCC)
#pragma config(Sensor, dgtl12, GreenLED,       sensorLEDtoVCC)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           IntakeLeft,    tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           DriveFrontLeft, tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           DriveFrontRight, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port4,           DriveBackLeft, tmotorVex393_MC29, PIDControl, encoderPort, I2C_1)
#pragma config(Motor,  port5,           DriveBackRight, tmotorVex393_MC29, PIDControl, reversed, encoderPort, I2C_2)
#pragma config(Motor,  port6,           ArmLeftOut,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           ArmLeftIn,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           ArmRightIn,    tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           ArmRightOut,   tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          IntakeRight,   tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)


#include "Vex_Competition_Includes.c"

//  Lucky Programming Clover
//           .-.
//          ( . )
//        .-.':'.-.
//       (  =,!,=  )
//        '-' | '-'

//
// 5D: Arm Down																6D: OUTTAKE
// 5U: Arm Up																	6U: INTAKE
//
//
//        				7U			         				    8U
//			Battery 7L  7R		         Morpheus 8L  8R Tank
//        				7D								          8D
//																				 Catapult
//
//								|									|
//						4	--+--							--+-- 1
//								|									|
//								3									2
//

// Quickness with which thejm  robot turns in response to right thumbstick
#define TURN_SPEED 2

#define driveThreshold 64
#define joystickThreshold 127

#define armSpeed 110
#define armMin 1260
#define armMax 2860
//#define armScore 2800

// Auton stuff
#define nOfAutons 2
#define autonMax 4095
#define autonMin 55

// PID
#define MAX_POWER 127
#define ARM_POWER 127
#define MAX_ARM 2200

//////////////////////
// SENSOR VARIABLES //
//////////////////////
int armRotation = SensorValue[ArmPotentiometer];
int encoderValue = 0;
int gyroValue = 0;
int autonRotation = SensorValue[AutonPotentiometer];
#define encoderMotor DriveBackRight

int armSpeedPrevious = 0;

// State of drive orientation: 0 = tank, 1 = x-drive
int morpheus = 0;

int oldY;

int channel1 = 0, channel3 = 0, channel4 = 0;
int isBeingIntaken;

///////////////////////
// TASK DECLARATIONS //
///////////////////////
task PID();
task Display_LCD();
task checkArmMovement();
task armMovement();

////////////////////
// HELPER METHODS //
////////////////////
void setMorpheus() { SensorValue[MorpheusL] = SensorValue[MorpheusR] = 1; }
void setTank() { SensorValue[MorpheusL] = SensorValue[MorpheusR] = 0; }

/////////////////
//// SENSORS ////
/////////////////
void updateSensors() {
  armRotation = SensorValue[ArmPotentiometer];
  encoderValue = nMotorEncoder[encoderMotor];
  gyroValue = SensorValue[Gyroscope];
  autonRotation = SensorValue[AutonPotentiometer];
}

// Resets all sensors:
// - Encoders
// - Gyro
void resetSensors() {
	nMotorEncoder[DriveBackLeft] = nMotorEncoder[DriveBackRight] = 0;
	SensorValue[Gyroscope] = 0;
}

void waitForButton() {
	while(SensorValue[AutonButton] == 0) {
		wait1Msec(10);
	}
}

////////////////////
///// MOVEMENT /////
////////////////////

// Uses a cubic function to make the acceleration smooth and not jerky or sudden.
// x is the input value of the joystick, return is the y value mapped along a cubic map
int cubicMap(int x) {
	return ((((x*3)/25)*((x*3)/25)*((x*3)/25)/27 + x/2)*2)/3;
}

void powerDrive(int power) {
	motor[DriveBackLeft] = motor[DriveFrontLeft] = motor[DriveBackRight] = motor[DriveFrontRight] = power;
}

// Base Movement Function
// c1 turns, c3 forward, c4 strafes
void move(int c1, int c3, int c4) {
	if(morpheus == 0) { // tank drive
		motor(DriveFrontLeft) = motor(DriveBackLeft) = (c3 + TURN_SPEED*channel1);
	  motor(DriveFrontRight) = motor(DriveBackRight) = (c3 - TURN_SPEED*channel1);
  } else if (morpheus == 1) // x-drive
  	motor(DriveFrontLeft) = c3 + c4 + c1;
		motor(DriveFrontRight) = c3 - c4 - c1;
		motor(DriveBackLeft) = c3 - c4 + c1;
		motor(DriveBackRight) = c3 + c4 - c1;
}

void UC_drive() {
	channel1 = cubicMap(vexRT[Ch1]);
	channel3 = cubicMap(vexRT[Ch3]);
	channel4 = cubicMap(vexRT[Ch4]);

	move(channel1, channel3, channel4);
}

//////////////////
///// INTAKE /////
//////////////////

void setIntake(int isIntake) {
	motor[IntakeLeft] = motor[IntakeRight] = 127 * isIntake;
}

void intakeT(int isIntake, int time) {
	setIntake(isIntake);
	wait1Msec(time);
	motor(IntakeLeft) = motor(IntakeRight) = 0;
}

void UC_intake() {
	if(vexRT[Btn6U])
		setIntake(1);
	else if (vexRT[Btn6D])
		setIntake(-1);
	else
		setIntake(0);
}

/////////////////////
//////// ARM ////////
/////////////////////

// Powers all arm motors.
void powerArm(int speed) {
	motor(ArmLeftIn) = motor(ArmLeftOut) = motor(ArmRightIn) = motor(ArmRightOut) = speed;
}

// Brings the arm to a gradual stop.
void stopArm() {
	if (armSpeedPrevious > 10) {
		armSpeedPrevious -= 1;
		powerArm(armSpeedPrevious);
	} else if(armSpeedPrevious < -10) {
		armSpeedPrevious += 1;
		powerArm(armSpeedPrevious);
	}
}

// Controls the algorithm in determining arm speed
void moveArm(int target) {
	if(target > 0 && armRotation < armMax) {
		// Raise Arm
		int tempSpeed = armSpeedPrevious + ((127-armSpeedPrevious)/5);
		powerArm(tempSpeed);
		armSpeedPrevious = tempSpeed;
	} else if (target < 0 && armRotation > armMin) {
		// Lower Arm
		int tempSpeed = armSpeedPrevious - ((127+armSpeedPrevious)/5);
		powerArm(tempSpeed);
		armSpeedPrevious = tempSpeed;
	} else {
		stopArm();
	}
}

// Arm movement
void UC_arm() {
	if (vexRT[Btn5U] == 1 && vexRT[Btn5D] == 1)
		moveArm(0);
	else if (vexRT[Btn5U] == 1)
	  moveArm(127);
	else if (vexRT[Btn5D] == 1)
	  moveArm(-127);
	else
		moveArm(0);
}

// Autonomous raising to a particular target
void raiseArm(int target) {
	while(armRotation < target) {
		powerArm(armSpeed);
		updateSensors();
	}
}
void lowerArm(int target) {
	while(armRotation > target) {
		powerArm(-armSpeed);
		updateSensors();
	}
}

void holdArm() {
	powerArm(30);
}

int targetArm;
int ARM_BUFFER;
bool isArm = false;

task armMovement() {
	while(true) {
		if(isArm) {
			if(armRotation < targetArm - ARM_BUFFER)
				powerArm(ARM_POWER);
			else if (armRotation > targetArm + ARM_BUFFER)
				powerArm(-ARM_POWER);
		}
	}
}

////////////////////////
// NON PID AUTONOMOUS //
////////////////////////
void forward(int distance) {
	resetSensors();
	while(nMotorEncoder[DriveBackLeft] < distance) {
		motor[DriveBackLeft] = motor[DriveBackRight] = 100;
		motor[DriveFrontLeft] = motor[DriveFrontRight] = 100;
	}
	motor[DriveBackLeft] = motor[DriveBackRight] = 0;
	motor[DriveFrontLeft] = motor[DriveFrontRight] = 0;
}

void back(int distance) {
	resetSensors();
	while(nMotorEncoder[DriveBackLeft] > -distance) {
		motor[DriveBackLeft] = motor[DriveBackRight] = -100;
		motor[DriveFrontLeft] = motor[DriveFrontRight] = -100;
	}
	motor[DriveBackLeft] = motor[DriveBackRight] = 0;
	motor[DriveFrontLeft] = motor[DriveFrontRight] = 0;
}

void left() {
	resetSensors();
	motor[DriveBackLeft] = motor[DriveFrontLeft] = -100;
	motor[DriveBackRight] = motor[DriveFrontRight] = 100;
	while((SensorValue[Gyroscope]) < 870) {
	}
	move(0, 0, 0);
}

void right() {
	resetSensors();
	motor[DriveBackLeft] = motor[DriveFrontLeft] = 100;
	motor[DriveBackRight] = motor[DriveFrontRight] = -100;
	while((SensorValue[Gyroscope]) > -870) {
	}
	move(0, 0, 0);
}

/////////////////////
//////// PID ////////
/////////////////////

float kp = 1;//0.5;

int targetDist = 2100;

int error;

int rampDown = 500;
int rampUp = 600;
int output, input;
int rampDownBasePower = 25;
int rampUpBasePower = 70;

int movementN;
bool isForwardMotion = true;
//bool isNextMovement = false;

bool isPID = true;

// Straight-line equation for ramping down, with basePower being the lowest it goes before hitting 0.
float getOutputByRampDown() {
	return (MAX_POWER - rampDownBasePower)/rampDown * output + (error)/(abs(error)) * rampDownBasePower;
}
float getOutputByRampUp() {
	return (MAX_POWER - rampUpBasePower)/rampUp * input + (error)/(abs(error)) * rampUpBasePower;
}

// Basic PID
// As of now, only P
task PID() {
	movementN = 0;
	while(true) {
		if(isPID) {
			error = targetDist - nMotorEncoder[encoderMotor];
			input = nMotorEncoder[encoderMotor] * kp;
			output = error * kp;
			if (nMotorEncoder[encoderMotor] < rampUp) {
				powerDrive(getOutputByRampUp());
			} else if (error < rampDown) {
				powerDrive(getOutputByRampDown());
			} else {
				powerDrive(MAX_POWER);
			}
			if(isForwardMotion == (error < 0))
			{ isPID = false;
				movementN ++; }
		}
	}
}

////////////////////////
// AUTONOMOUS HELPERS //
////////////////////////

void stopMoving() {
	move(0, 0, 0);
}

void stopEverything() {
	powerArm(0);
	move(0, 0, 0);
}

task checkArmMovement() {
	int dArm = 0;
	while(true) {
		dArm = SensorValue[ArmPotentiometer] - armRotation;
		if(abs(dArm) < 10 && abs(motor[ArmLeftIn]) > 50) {
			motor[ArmLeftIn] = motor[ArmLeftOut] = 0;
		}
	}
}

////////////////////////
// AUTONOMOUS METHODS //
////////////////////////

void auton0() {
	for(int i = 0; i < 4; i++) {
		forward(500);
		wait1Msec(500);
		right();
		wait1Msec(500);
	}
}

void auton1() {
	setIntake(-1);
	wait1Msec(200);
	raiseArm(2430);
	holdArm();
	forward(670);
	stopMoving();
	wait1Msec(200);
	back(690);
	stopMoving();
	// Button
	waitForButton();
	forward(1400);
	move(0, 40, 0);
	move(0, -40, 0);
	move(0, 0, 0);
	wait1Msec(1000);
	back(1420);
	stopEverything();
}

void auton2() {
	forward(2540);
	raiseArm(armMax);
	holdArm();
	forward(340);
	setIntake(-1);
	wait1Msec(1500);
	back(300);
	lowerArm(armMin);
}

void autonomousMethod()
{
	pre_auton();
	setTank();
	wait1Msec(1000);

	startTask(checkArmMovement);
	startTask(PID);

	/*
	if(autonRotation < 2048) {
		auton1();
	} else {
		auton2();
	}*/
}

///////////
// DEBUG //
///////////
void mainDisplay() {
	string s;
	s = rampDownBasePower;
  displayLCDCenteredString(0, "rampDownBasePower");
  displayLCDCenteredString(1, s);
}

task Display_LCD(){
  bLCDBacklight = true;
  while(true){
    // Battery level indicator
    string display = "R:";
  	string regularBat, backupBat;
    StringFormat(regularBat, "%1.2f", (float)nImmediateBatteryLevel/1000);
    StringFormat(backupBat, "%1.2f", (float)BackupBatteryLevel/1000);
    StringConcatenate(display, regularBat);
    StringConcatenate(display, "V B:");
    StringConcatenate(display, backupBat);
    StringConcatenate(display, "V ");

    clearLCDLine(0);
    clearLCDLine(1);

    if(vexRT[Btn7L] == 1) {
    	displayLCDCenteredString(0, "Battery Power:");
    	displayLCDCenteredString(1, display);
  	} else {
    	if((float)nImmediateBatteryLevel/1000 < 6.45) {
    		displayLCDCenteredString(0, "***WARNING***");
    		displayLCDCenteredString(1, "*LOW BATTERY*");
    	} else
    		mainDisplay();
    }
    wait1Msec(100);
  }
}

///////////////////////////
// LCD + GENERAL BUTTONS //
///////////////////////////
void rightLcdClicked() {
	rampDownBasePower += 5;
}

void leftLcdClicked() {
	rampDownBasePower -= 5;
}

void centerLcdClicked() {
}

// Default LCD parsing code
int oldLCDbuttons = 0;
task lcdButtons() {
	if (nLCDButtons != oldLCDbuttons) {
		if(nLCDButtons == 4) {
			rightLcdClicked();
		} else if (nLCDButtons == 1) {
			leftLcdClicked();
		} else if (nLCDButtons == 2) {
			centerLcdClicked();
		}
	}
}

void UC_buttons() {
	if(vexRT[Btn8L]) {
		setMorpheus();
  } else if (vexRT[Btn8R]) {
		setTank();
	}

	if(vexRT[Btn8D]) {
		SensorValue[Catapult] = 1;
	} else {
		SensorValue[Catapult] = 0;
	}

	if(vexRT[Btn7U]) {

	}

	if(SensorValue[AutonButton] == 1) {
		autonomousMethod();
	}
}

/////////////////////////
// COMPETITION METHODS //
/////////////////////////

// All activities that occur before the competition starts
void pre_auton()
{
  bStopTasksBetweenModes = true;
  // User-defined code
  intakeT(1, 1500);
	resetSensors();
}

task autonomous()
{
	startTask(Display_LCD);
	autonomousMethod();
}

task usercontrol()
{
	resetSensors();
	startTask(Display_LCD);
	startTask(lcdButtons);
	//motor[Flash] = 127;
	while (true) {
		oldLCDbuttons = nLCDButtons;
		updateSensors();

		UC_drive();
		UC_arm();
		UC_intake();
		UC_buttons();
	}
}
