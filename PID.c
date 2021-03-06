#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    ArmPotentiometer, sensorPotentiometer)
#pragma config(Sensor, in2,    Gyro,           sensorGyro)
#pragma config(Sensor, in3,    LeftLine,       sensorLineFollower)
#pragma config(Sensor, in5,    RightLine,      sensorLineFollower)
#pragma config(Sensor, dgtl1,  Morpheus,       sensorDigitalOut)
#pragma config(Sensor, dgtl2,  Catapult,       sensorDigitalOut)
#pragma config(Sensor, dgtl12, Button,         sensorTouch)
#pragma config(Sensor, I2C_1,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_3,  ,               sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           IntakeLeft,    tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port2,           DriveFrontLeft, tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port3,           DriveFrontRight, tmotorVex393HighSpeed, openLoop, reversed)
#pragma config(Motor,  port4,           DriveBackLeft, tmotorVex393HighSpeed, PIDControl, encoder, encoderPort, I2C_1, 1000)
#pragma config(Motor,  port5,           DriveBackRight, tmotorVex393HighSpeed, PIDControl, reversed, encoder, encoderPort, I2C_2, 1000)
#pragma config(Motor,  port6,           DriveCenter,   tmotorVex393HighSpeed, openLoop)
#pragma config(Motor,  port7,           Arm1,          tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port8,           Arm2,          tmotorVex393, PIDControl, encoder, encoderPort, I2C_3, 1000)
#pragma config(Motor,  port9,           Arm3,          tmotorVex393, openLoop, reversed)
#pragma config(Motor,  port10,          IntakeRight,   tmotorVex393, openLoop)

//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(105)

#define encoderMotor DriveBackLeft
#define MAX_POWER 127
#define rampDownBasePower 12
#define rampUpBasePower 50
#define ARM_POWER 127
#define ARM_BUFFER 60
#define MAX_ARM 2200

float kp = 0.5;

int targetDist = 2100;
int targetArm = 2100;

int error;
int rampDown = 400;
int rampUp = 600;
int output;

int armRotation;

task PID();
task setArm();

void resetEncoders() {
	nMotorEncoder[DriveBackLeft] = nMotorEncoder[DriveBackRight] = 0;
}

void powerMotors(int power) {
	motor[DriveBackLeft] = motor[DriveFrontLeft] = power;
	motor[DriveBackRight] = motor[DriveFrontRight] = power;
}

// Straight-line equation for ramping down, with basePower being the lowest it goes before hitting 0.
float getOutputByRampDown() {
	return 107.0/rampDown * output + (error)/(abs(error)) * rampDownBasePower;
}
float getOutputByRampUp() {
	return 67.0/rampUp * output + (error)/(abs(error)) * rampUpBasePower;
}

int getSign(int x) {
	return x/abs(x);
}

task straight() {
	int left, right;
	while(true) {
		left = nMotorEncoder[DriveBackLeft];
		right = abs(nMotorEncoder[DriveBackRight]);
		if(left < right) {

		} else if (left > right) {

		}
	}
}

void updateSensors() {
	armRotation = SensorValue[ArmPotentiometer];
}

void powerArm(int x) {
	motor[Arm1] = motor[Arm2] = motor[Arm3] = x;
}

task setArm() {
	updateSensors();
	while(true) {
		if(armRotation < targetArm) {
			powerArm(ARM_POWER);
		} else if (armRotation > targetArm) {
			powerArm(-ARM_POWER);
		}
	}
}

// Basic PID
// As of now, only P
task PID() {
	while(true) {
		error = targetDist - nMotorEncoder[encoderMotor];
		output = error * kp;
		if (nMotorEncoder[encoderMotor] < rampUp) {
			powerMotors(getOutputByRampUp());
		} else if (error < rampDown) {
			powerMotors(getOutputByRampDown());
		} else {
			powerMotors(MAX_POWER);
		}
	}
}

task main() {
	resetEncoders();
	StartTask(PID);
	StartTask(setArm);
	while(true) { }
}
