/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      10/10/2025, 4:53:21 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
// A global instance of competition
competition Competition;

// 0 = left
// 1 = right
// 2 = skills
int autonToggle = 1; //0, 1

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;
inertial Gyro = inertial(PORT16);
//drivebase
motor leftFront = motor(PORT5, ratio6_1, false);
motor leftMiddle = motor(PORT6, ratio18_1, true);
motor leftBack = motor(PORT7, ratio6_1, false);
motor rightFront = motor(PORT11, ratio6_1, true);
motor rightMiddle = motor(PORT1, ratio18_1, false);
motor rightBack = motor(PORT2, ratio6_1, true);
motor_group leftSide = motor_group(leftFront, leftMiddle, leftBack);
motor_group rightSide = motor_group(rightFront, rightMiddle, rightBack);

//intakes
motor rollersBottom = motor(PORT12, ratio6_1, true); // rollersBottom
motor rollersTop = motor(PORT17, ratio6_1, false); // rollersTop
motor topIntake = motor(PORT3, ratio6_1, false);
  
//pneumatics
digital_out descore = digital_out(Brain.ThreeWirePort.G);
digital_out matchLoader = digital_out(Brain.ThreeWirePort.H);
digital_out stopPiston = digital_out(Brain.ThreeWirePort.E);
digital_out frontDescore = digital_out(Brain.ThreeWirePort.F);

// pid constants
double pi = 3.1415926;
double diameter = 3.25;
double g = 36.0/48.0;

// piston controls
void stopPistonControl() {
  stopPiston.set(!stopPiston.value());
}

void loaderControl() {
  matchLoader.set(!matchLoader.value());
}

void descoreControl() {
  descore.set(!descore.value());
}

void frontDescoreControl() {
  frontDescore.set(!frontDescore.value());
}

void blockholderControl() {
  blockholder.set(!blockholder.value());
}

void drive(int lspeed, int rspeed, int wt) {
  leftSide.spin(forward, lspeed, pct);
  rightSide.spin(forward, rspeed, pct);
  wait(wt, msec);
}

//auton functions
// intake to the top goal
void intakeTop() {
  rollersBottom.spin(reverse, 100, pct);
  rollersTop.spin(forward, 100, pct);
  topIntake.spin(forward, 90, pct);
}
// intake to the bottom middle goal
void intakeBottomMiddle() {
  rollersBottom.spin(reverse, 100, pct);
  rollersTop.spin(forward, 100, pct);
  topIntake.spin(reverse, 100, pct);
}
// intake to the bottom bottom goal
void intakeBottomBottom() {
  rollersBottom.spin(forward, 35, pct);
  rollersTop.spin(reverse, 100, pct);
  topIntake.spin(reverse, 100, pct);
}
// stop all intakes
void intakeStop() {
  rollersBottom.stop();
  rollersTop.stop();
  topIntake.stop();
}

void DriveVolts(double lspeed, double rspeed, double multipier, int wt) {
  lspeed = lspeed * 120 * multipier;
  rspeed = rspeed * 120 * multipier;
  leftFront.spin(forward, lspeed, voltageUnits::mV);
  leftMiddle.spin(forward, lspeed, voltageUnits::mV);
  leftBack.spin(forward, lspeed, voltageUnits::mV);
  rightFront.spin(forward, rspeed, voltageUnits::mV);
  rightMiddle.spin(forward, rspeed, voltageUnits::mV);
  rightBack.spin(forward, rspeed, voltageUnits::mV);
}

void inchDrive(float target, int timeout = 1200, float kp = 4.5) {
  timer t2;
  t2.reset();
  float x = 0.0;
  float ap = 2.5; // increase = more wiggle // decrease = less wiggle 3
  float aerror = 0;
  float Atarget = Gyro.rotation();
  float heading = Gyro.rotation();
  float aspeed = 0;
  float tolerance = 1;
  float accuracy = 1;
  float ki = 0;
  float error = target - x;
  float speed = error * kp;
  float integral = 0;
  float prevError = target;
  float derivative = 0;
  float kd = 2;
  rightFront.setPosition(0.0, rev);
  while (t2.time(msec) < timeout) {
    heading = Gyro.rotation();
    aerror = Atarget - heading;
    aspeed = ap*aerror;
    x = rightFront.position(rev) * pi * diameter * g;
    error = target - x;
    if (fabs(error) < tolerance) {
      integral += error;
    }
    if(speed >= 100){
      speed = 100;
    }
    if (speed <= -100){
      speed = -100;
    }
    derivative = error - prevError;
    prevError = error;
    speed = error * kp + integral * ki + derivative * kd;
    DriveVolts(speed+aspeed, speed-aspeed, 1, 10);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.clearLine();
    Controller1.Screen.print(error);
  }
  leftSide.setStopping(brake);
  rightSide.setStopping(brake);
  wait(10, msec);
}
  
void gyroturnAbs(double target, int timeout = 1200) {
    timer t1;
    t1.reset();
    float kp = 1.75;
    float ki = 0;
    float kd = 0.6;
    float integral = 0;
    float integralTolerance = 3;
    // float integralMax = 100;
    float heading = 0.0;
    float error = target - heading;
    float prevError = 0;
    float derivative;
    float speed = kp * error;
    float accuracy = 0.1;
    float bias = 0;
    int count = 0;
  
  
    while (t1.time(msec) < timeout)
    {
      heading = Gyro.rotation(degrees);
      error = target - heading;
      derivative = (error - prevError);
      prevError = error;
      if (fabs(error) < integralTolerance)
      {
        integral += error;
      }
      if (fabs(error) < accuracy)
      {
        count++;
      }
      else {
        count = 0;
      }
      if (count > 20) {
        break;
      }
      speed = kp * error + kd * derivative + ki * integral;
      DriveVolts(speed, -speed, 1, 0);
    }
    leftSide.setStopping(brake);
    rightSide.setStopping(brake);
    wait(10, msec);
  }


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Gyro.calibrate();
  // while (Gyro.isCalibrating()) {
  //   wait(20, msec);
  // }z
  // wait(200, msec);
  // Gyro.setHeading(0, deg);
  // Gyro.setRotation(0, deg);
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  switch(autonToggle) {
    case 0: // left side
      intakeTop();
      stopPiston.set(true);
      inchDrive(27.5);
      gyroturnAbs(-31.5, 800);
      inchDrive(13, 900, 2.7);
      gyroturnAbs(-120);
      inchDrive(32);
      matchLoader.set(true);
      gyroturnAbs(-180);
      inchDrive(15);
      wait(600, msec);
      rollersBottom.stop();
      rollersTop.stop();
      topIntake.stop();
      stopPiston.set(false);
      frontDescore.set(true);
      inchDrive(-26);
      matchLoader.set(false);
      intakeTop();
      break;
    case 1: // right side (works prety much)
      intakeTop();
      stopPiston.set(true);
      inchDrive(26);
      gyroturnAbs(31.5, 800);
      inchDrive(13, 900, 2.7);
      gyroturnAbs(120);
      inchDrive(31.5); // goijg to goal
      matchLoader.set(true);
      gyroturnAbs(185);
      inchDrive(17, 800, 5);
      wait(400, msec);
      frontDescore.set(true);
      rollersBottom.stop();
      rollersTop.stop();
      topIntake.stop();
      stopPiston.set(false);
      inchDrive(-27, 1200, 3.8);
      matchLoader.set(false);
      intakeTop();
      break;
    /*
    case 0: // left side
      intakeTop();
      stopPiston.set(true);
      inchDrive(24.5);
      gyroturnAbs(-28);
      inchDrive(14, 700, 3.2);
      gyroturnAbs(-135);
      break;
    case 1: // right side
      intakeTop();
      stopPiston.set(true);
      inchDrive(26);
      gyroturnAbs(28, 500);
      inchDrive(18, 700, 3.2);
      intakeStop();
      wait(300, msec);
      gyroturnAbs(-47, 800);
      intakeBottomBottom();
      inchDrive(13.5, 500);
      wait(1000, msec);
      intakeStop();
      inchDrive(-41);
      matchLoader.set(true);
      gyroturnAbs(-169);
      inchDrive(-20, 1000);
      intakeTop();
      inchDrive(29, 1200, 4.0);
      wait(550, msec);
      intakeStop();
      inchDrive(-26);
      frontDescore.set(true);
      stopPiston.set(false);
      intakeTop();
      break;
      /
    // case 2: // SKILLS
    //   break;
    */
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  Controller1.ButtonDown.pressed(loaderControl);
  Controller1.ButtonRight.pressed(stopPistonControl);
  Controller1.ButtonY.pressed(frontDescoreControl);
  Controller1.ButtonB.pressed(descoreControl);
  Controller1.Button
  while (1) {
    double sensitivity = 1;
    int leftSpeed = (Controller1.Axis3.position(pct) + Controller1.Axis1.position(pct)) * sensitivity;
    int rightSpeed = (Controller1.Axis3.position(pct) - Controller1.Axis1.position(pct)) * sensitivity;
    drive (leftSpeed, rightSpeed, 10);
    // scoring top
    if (Controller1.ButtonR1.pressing()) {
      rollersBottom.spin(reverse, 100, pct);
      rollersTop.spin(forward, 100, pct);
      topIntake.spin(forward, 100, pct);
    }
    // scoring bottom
    else if (Controller1.ButtonR2.pressing()) {
      rollersBottom.spin(reverse, 100, pct);
      topIntake.spin(reverse, 100, pct);
      rollersTop.spin(forward, 60, pct);
    }
    // descoring
    else if (Controller1.ButtonL1.pressing()) {
      rollersBottom.spin(forward, 100, pct);
      rollersTop.spin(reverse, 100, pct);
    }
    else {
      rollersBottom.stop();
      topIntake.stop();
      rollersTop.stop();
    }
    wait(20, msec);
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}