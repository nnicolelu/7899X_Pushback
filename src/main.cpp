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

// auton toggles: 
// 0 = left blue (too close to the wall)
// 1 = left red regular GOOD -------
// 2 = right red (too close to the wall)
// 3 = right blue regular GOOD ---------
// 4 = skills
int autonToggle = 4;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;
inertial Gyro = inertial(PORT16);
//drivebase
motor leftFront = motor(PORT5, ratio6_1, false);
motor leftMiddle = motor(PORT6, ratio6_1, false);
motor leftBack = motor(PORT7, ratio6_1, false);
motor rightFront = motor(PORT11, ratio6_1, true);
motor rightMiddle = motor(PORT1, ratio6_1, true);
motor rightBack = motor(PORT2, ratio6_1, true);
motor_group leftSide = motor_group(leftFront, leftMiddle, leftBack);
motor_group rightSide = motor_group(rightFront, rightMiddle, rightBack);

//intakes
motor rollersBottom = motor(PORT12, ratio6_1, true); // rollersBottom
motor rollersTop = motor(PORT17, ratio18_1, false); // rollersTop
motor topIntake = motor(PORT3, ratio18_1, false);
  
//pneumatics
digital_out descore = digital_out(Brain.ThreeWirePort.G);
digital_out matchLoader = digital_out(Brain.ThreeWirePort.H);
digital_out stopPiston = digital_out(Brain.ThreeWirePort.F); // false = open true = close
//digital_out frontDescore = digital_out(Brain.ThreeWirePort.F);

// constants
double pi = 3.1415926;
double diameter = 3.25;
double g = 36.0/48.0;
double width = ;
color blue = color(0, 0, 255);
color red = color(255, 0, 0);

// piston controls
void loaderControl() {
  matchLoader.set(!matchLoader.value());
}

void descoreControl() {
  descore.set(!descore.value());
}

void unloading() {
  stopPiston.set(!stopPiston.value());
}

// stop all rollers
void stopAll() {
  rollersBottom.stop();
  rollersTop.stop();
  topIntake.stop();
}

// stop top rollers
void stopTop() {
  rollersTop.stop();
  topIntake.stop();
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
  rollersTop.spin(forward, 80, pct);
  topIntake.spin(forward, 100, pct);
}
// intake to the middle top goal
void intakemiddleTop() {
  rollersBottom.spin(reverse, 100, pct);
  rollersTop.spin(forward, 100, pct);
  topIntake.spin(reverse, 70, pct);
}
// intake to the middle bottom goal
void intakeMiddleBottom() {
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

void inchDrive(float target, int timeout = 1200, float kp = 3.8) {
  timer t2;
  t2.reset();

  float x = 0.0;
  float ap = 2.5; // increase = more wiggle // decrease = less wiggle 3
  float aerror = 0;
  float Atarget = Gyro.rotation();
  float heading = Gyro.rotation();
  float aspeed = 0.0;
  const float tolerance = 1.0;
  const float accuracy = 1.0; //may need adjust
  int count = 0;

  const float ki = 0.0;
  const float kd = 2.65;
 
  float integral = 0;
  float prevError = target;
  float derivative = 0;
  
  float error = target - x;
  float speed = error * kp;
 
  leftFront.setPosition(0.0, rev);
  rightFront.setPosition(0.0, rev);
  
  while (t2.time(msec) < timeout) {
    heading = Gyro.rotation();
    aerror = Atarget - heading;
    aspeed = ap*aerror;
  
    x = ((rightFront.position(rev) + leftFront.position(rev)) / 2.0) * pi * diameter * g;
    error = target - x;
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
  
    if (fabs(error) < tolerance) {
      integral += error;
    }
    
    speed = error * kp + integral * ki + derivative * kd;
    
    if(speed >= 100){
      speed = 100;
    }
    
    if (speed <= -100){
      speed = -100;
    }
    
    derivative = error - prevError;
    prevError = error;
    DriveVolts(speed+aspeed, speed-aspeed, 1, 10);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.clearLine();
    Controller1.Screen.print(error);
    wait(10, msec);
  }
  leftSide.stop(brake);
  rightSide.stop(brake);
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
    while (t1.time(msec) < timeout) {
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
      wait(10, msec);
    }
    leftSide.setStopping(brake);
    leftSide.stop();
    rightSide.setStopping(brake);
    rightSide.stop();
    wait(10, msec);
  }
// rd = radius of circular path
void arcTurn(float rd, float angle, float maxSpeed = 100) {
    float kp = 1.0;
    float kd = 1.0;
    float targetArcLength = rd * 2 * pi * (angle/360.0);
    float arcLength = 0.0;
    float error = targetArcLength - arcLength;
    float oldError = error;
    float lspeed = (maxSpeed * angle) / fabs(angle);
    float rspeed = (lspeed * (rd - width)) / rd;
    float accuracy = 0.2;
    leftMiddle.setPosition(0.0, rev);
    rightMiddle.setPosition(0.0, rev);
    while (fabs(error) >= accuracy) {
      DriveVolts(lspeed, rspeed, 1, 10);
      arcLength = leftMiddle.position(rev) * g * pi * diameter;
      oldError = error;
      error = targetArcLength - arcLength;
      lspeed = (kp * error) + (kd * (error-oldError));
      if (fabs(lspeed) >= maxSpeed) {
        lspeed = (maxSpeed * error) / fabs(error);
        rspeed = (lspeed * (rd - width)) / rd;
      }
      leftSide.setStopping(brake);
      rightSide.setStopping(brake);
      leftSide.stop();
      rightSide.stop();
    }
    
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
  Gyro.calibrate();
  while (Gyro.isCalibrating()) {
    wait(20, msec);
  }
  wait(200, msec);
  Gyro.setHeading(0, deg);
  Gyro.setRotation(0, deg);
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
    case 0: // left side too close to wall
      intakeTop();
      stopPiston.set(false);
      inchDrive(27);
      gyroturnAbs(-25.5, 800);
      inchDrive(14, 900, 3); // 2.7
      gyroturnAbs(-120);
      inchDrive(31.5);
      matchLoader.set(true);
      gyroturnAbs(-185);
      inchDrive(18, 800, 4.2); // match loading
      wait(470, msec);
      stopAll();
      stopPiston.set(true);
      inchDrive(-10, 1200, 3.8);
      gyroturnAbs(-200);
      inchDrive(-20, 1200, 3.8);
      matchLoader.set(false);
      intakeTop();
      break;
    case 1: // left side regular
      intakeTop();
      stopPiston.set(false);
      inchDrive(27);
      gyroturnAbs(-25, 800);
      inchDrive(14, 900, 3); // 2.7
      gyroturnAbs(-120);
      inchDrive(34); // driving to match loader
      matchLoader.set(true);
      gyroturnAbs(-185);
      inchDrive(18, 800, 4.2); // match loading
      wait(410, msec);
      stopAll();
      stopPiston.set(true);
      inchDrive(-31, 1200, 3.8);
      //matchLoader.set(false);
      intakeTop();
      break;
    case 2: // right red side (works prety much)
      intakeTop();
      stopPiston.set(false);
      inchDrive(27);
      gyroturnAbs(28.5, 800);
      inchDrive(14, 900, 3); // 2.7
      gyroturnAbs(120);
      inchDrive(33); // goijg to goal
      matchLoader.set(true);
      gyroturnAbs(180);
      inchDrive(20, 800, 4.2); // match loading
      wait(450, msec);
      stopAll();
      stopPiston.set(true);
      inchDrive(-30, 1200, 3.8);
      matchLoader.set(false);
      intakeTop();
      break;
    case 3: // right blue side (use this one)
      intakeTop();
      stopPiston.set(false);
      inchDrive(27);
      gyroturnAbs(28.5, 800);
      inchDrive(14, 900, 3);
      gyroturnAbs(120);
      inchDrive(33.5); // goijg to goal
      matchLoader.set(true);
      gyroturnAbs(178);
      inchDrive(18, 800, 4.2); // match loading
      wait(480, msec); // need to lessen this mayb
      stopAll();
      stopPiston.set(true);
      inchDrive(-30, 1200, 3.8);
      matchLoader.set(false);
      intakeTop();
      break;
    case 4: // SKILLS
      intakeTop();
      stopPiston.set(false);
      inchDrive(21, 800);
      gyroturnAbs(34, 600);
      inchDrive(20, 800);
      gyroturnAbs(130, 850); // 900
      inchDrive(32, 950); // drive to goal // 1000
      matchLoader.set(true);
      gyroturnAbs(180, 800); // turn to match load
      inchDrive(18, 2100, 4.2);
      wait(200, msec);
      stopTop();
      stopPiston.set(true);
      inchDrive(-32, 1500, 3.8);
      intakeTop(); // end first goal
      wait(2000, msec);
      stopAll();
      inchDrive(20, 800); 
      gyroturnAbs(70); // can be turned into arc turn
      inchDrive(25); // can be turned into arc turn
      gyroturnAbs(90); // can be turned into arc turn
      break;
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
  Controller1.ButtonY.pressed(unloading);
  Controller1.ButtonB.pressed(descoreControl);
  //Controller1.Button
  while (1) {
    Brain.Screen.print("bottom voltage: ");
    Brain.Screen.print(rollersBottom.voltage(volt));
    Brain.Screen.newLine();
    Brain.Screen.print("top voltage: ");
    Brain.Screen.print(rollersTop.voltage(volt));
    Brain.Screen.newLine();
    double sensitivity = 1;
    int leftSpeed = (Controller1.Axis3.position(pct) + Controller1.Axis1.position(pct)) * sensitivity;
    int rightSpeed = (Controller1.Axis3.position(pct) - Controller1.Axis1.position(pct)) * sensitivity;
    drive (leftSpeed, rightSpeed, 10);
    // scoring top top
    if (Controller1.ButtonR1.pressing()) {
      rollersTop.spin(forward, 100, pct);
      topIntake.spin(forward, 100, pct);
      if (Controller1.ButtonL1.pressing()) {
        rollersBottom.spin(reverse, 100, pct);
      }
      else {
        rollersBottom.stop();
      }
    }
    // scoring bottom top
    else if (Controller1.ButtonR2.pressing()) {
      rollersTop.spin(forward, 100, pct);
      topIntake.spin(reverse, 70, pct);
    }
    // rotating the bottom rollers
    else if (Controller1.ButtonL1.pressing()) {
      rollersBottom.spin(reverse, 100, pct);
      // scoring on the top
      if (Controller1.ButtonR1.pressing()) {
        rollersTop.spin(forward, 80, pct);
        topIntake.spin(forward, 100, pct);
      } // scoring bottom top
      else if (Controller1.ButtonR2.pressing()) {
        rollersTop.spin(forward, 100, pct);
        topIntake.spin(reverse, 100, pct);        
      }
      else {
        rollersTop.stop();
        topIntake.stop();
      }
    }
    // remove balls out of robot
    else if (Controller1.ButtonL2.pressing()) {
      rollersTop.spin(reverse, 100, pct);
      rollersBottom.spin(forward, 100, pct);
      topIntake.spin(reverse, 100, pct);
    }
    else {
      rollersBottom.stop();
      topIntake.stop();
      rollersTop.stop();
    }
    /*
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
    // loop to stop the rollers when there are more than 4 balls 
    if ((stopPiston.value() == true) && Controller1.ButtonR1.pressing()) {
      int count = 0;
      while (count <= 4) {
        if (// color == blue or color == red) {
          rollersBottom.spin(reverse, 100, pct);
          rollersTop.spin(forward, 100, pct);
          topIntake.spin(forward, 100, pct);    
          count++;  
        }
      }
      rollersTop.stop();
      topIntake.stop();
    }
    */
    wait(10, msec);
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
    wait(10, msec); // 100
  }
}