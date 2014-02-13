/** @file opcontrol.c
 * @brief File for operator control code
 *
 * This file should contain the user operatorControl() function and any functions related to it.
 *
 * Copyright (c) 2011-2013, Purdue University ACM SIG BOTS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Purdue University ACM SIG BOTS nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL PURDUE UNIVERSITY ACM SIG BOTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Purdue Robotics OS contains FreeRTOS (http://www.freertos.org) whose source code may be
 * obtained from http://sourceforge.net/projects/freertos/files/ or on request.
 */

#include "main.h"
#include "help.c"

// Variables
int armRotation;
int morpheus; // state of drive orientation: 0 = tank, 1 = x-drive
int armSpeedPrevious;

/*
 * Runs the user operator control code. This function will be started in its own task with the
 * default priority and stack size whenever the robot is enabled via the Field Management System
 * or the VEX Competition Switch in the operator control mode. If the robot is disabled or
 * communications is lost, the operator control task will be stopped by the kernel. Re-enabling
 * the robot will restart the task, not resume it from where it left off.
 *
 * If no VEX Competition Switch or Field Management system is plugged in, the VEX Cortex will
 * run the operator control task. Be warned that this will also occur if the VEX Cortex is
 * tethered directly to a computer via the USB A to A cable without any VEX Joystick attached.
 *
 * Code running in this task can take almost any action, as the VEX Joystick is available and
 * the schedular is operational. However, proper use of delay() or taskDelayUntil() is highly
 * recommended to give other tasks (including system tasks such as updating LCDs) time to run.
 *
 * This task should never exit; it should end with some kind of infinite loop, even if empty.
 */
void operatorControl() {
	while (1) {
		delay(20);
	}
}

// Allocates the specified arm power to the arm motors
void powerArm(int speed) {
	motorSet(Arm1, speed);
	motorSet(Arm2, speed);
}



// Brings the arm to a gradual stop
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
	if(target > 0) {
		int tempSpeed = armSpeedPrevious + ((127-armSpeedPrevious)/5);
		powerArm(tempSpeed);
		armSpeedPrevious = tempSpeed;
	} else if (target < 0) {
		int tempSpeed = armSpeedPrevious - ((127+armSpeedPrevious)/5);
		powerArm(tempSpeed);
		armSpeedPrevious = tempSpeed;
	} else {
		stopArm();
	}
}


void UC_drive() {
	if (joystickGetDigital(1, 8, JOY_LEFT)) {
		morpheus = 1;
	} else if (joystickGetDigital(1, 8, JOY_LEFT)) {
		morpheus = 0;
	}

	// Pneumatics
	digitalWrite(RightMorpheus, morpheus);
	digitalWrite(LeftMorpheus, morpheus);

	// Basic movement
	int channel1 = cubicMap(joystickGetAnalog(1, 1));
	int channel3 = cubicMap(joystickGetAnalog(1, 3));
	int channel4 = cubicMap(joystickGetAnalog(1, 4));
	if (morpheus == 0) {
		//motor(LeftFrontMotor1) = motor(LeftBackMotor1) = motor(LeftBackMotor2) = channel3 + TURN_SPEED * channel1;
		//motor(RightFrontMotor1) = motor(RightBackMotor1) = motor(RightBackMotor2) = channel3 - TURN_SPEED * channel1;
		// tank drive
		float morpheus1 = channel3 + TURN_SPEED * channel1;
		motorSet(LeftFrontMotor1, morpheus1);
		motorSet(LeftBackMotor1, morpheus1);
		motorSet(LeftBackMotor2, morpheus1);

		float morpheus2 = channel3 - TURN_SPEED * channel1;
		motorSet(RightFrontMotor1, morpheus2);
		motorSet(RightBackMotor1, morpheus2);
		motorSet(RightBackMotor2, morpheus2);

	} else {
		// x-drive
		motorSet(LeftFrontMotor1, channel3 + channel4 + channel1);
		motorSet(RightFrontMotor1, channel3 - channel4 - channel1);
		motorSet(LeftBackMotor1, channel3 - channel4 + channel1);
		motorSet(LeftBackMotor2, channel3 - channel4 + channel1);
		motorSet(RightBackMotor1, channel3 + channel4 - channel1);
		motorSet(RightBackMotor2, channel3 + channel4 - channel1);
	}
}

void UC_arm(int safe) {
	//if (safe) {
	if (joystickGetDigital(1, 5, JOY_UP)
			&& joystickGetDigital(1, 5, JOY_DOWN)) {
		moveArm(0);
	} else if (joystickGetDigital(1, 5, JOY_UP)) {  	 // bring the arm up
		moveArm(127);
	} else if (joystickGetDigital(1, 5, JOY_DOWN)) {	 // bring the arm down
		moveArm(-127);
	} else {								// don't do anything to the arm
		moveArm(0);
	}
	/*} else {
	 if (vexRT[Btn5U] == 1 && vexRT[Btn5D] == 1) {
	 moveArm(0);
	 } else if (vexRT[Btn5U] == 1) {  	 // bring the arm up
	 moveArm(127);
	 } else if (vexRT[Btn5D] == 1) {	 // bring the arm down
	 moveArm(-127);
	 } else {						// don't do anything to the arm
	 moveArm(0);
	 }
	 }*/
}

void UC_intake() {
	if (joystickGetDigital(1, 6, JOY_UP)) {
		motorSet(Intake, 127);
	} else if (joystickGetDigital(1, 6, JOY_DOWN)) {
		motorSet(Intake, -127);
	} else {
		motorSet(Intake, 0);
	}
}

int toggle(int a) {
	return !a;
}

void updateSensors() {
	armRotation = analogRead(ArmPotentiometer);
}

