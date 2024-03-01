/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jessica                                                     */
/*    Created:      8/26/2023, 6:05:02 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "Conversion.h"
//#include "GlobalVariables.h"
//#include "Odometry.h"
//#include "PP.h"
#include "OdomOOP.h"
#include "PIDOOP.h"


using namespace vex;
task TrackTask;
task TrackGoal;
task TrackDriveP;
Odometry Odom;
PID turnPID;
PID OdomPID;
// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

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
  vexcodeInit();
  Back.resetPosition();
  Right.resetPosition();
  
  
  Odom.setXIn(2);
  Odom.setYIn(2);
  //Odom.setAng(M_PI/2);
  Odom.setRDistInput(1);//.7 .7
  Odom.setSDistInput(.8);//.3 1
  

  turnPID.setkD(.09);
  turnPID.setkI(.005);
  turnPID.setkP(.13);
  turnPID.setTurnThresh(15);
  turnPID.setTurnTolerance(.6);
  turnPID.setMaxSpeed(7);
  turnPID.setMaxIter(300);

  OdomPID.setiBoundT(.2);
  OdomPID.setkIT(.005);
  OdomPID.setkPT(.13);
  OdomPID.setKDT(.09);
  OdomPID.setMaxSpeedT(7);

  OdomPID.setiBound(9);
  OdomPID.setkD(.0009);
  OdomPID.setkI(.0001);
  OdomPID.setkP(.02);
  OdomPID.setMaxSpeed(10);


  //PathGens();
  //DistBP(NPoint,3);
  //Curve(NPoint,3);
  //Velocity(12);
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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

bool halfspeed = false;
bool soloControl = false;
//void solo() {soloControl = !soloControl;}

void halfspeedcontrol() { halfspeed = !halfspeed; }

void usercontrol(void) {

  int threshold = 20, ChasLfVar = 0, ChasRtVar = 0;
  
  
  //Controller1.ButtonA.pressed(halfspeedcontrol);

  
 
  while (1) {

    


    //Test Button for Autonmous
    //Empty for compeititon
    if(Controller1.ButtonX.pressing()){
      turnPID.turnPTo(-90);
    }
    if (Controller1.ButtonA.pressing()){
      //Drivetrain.driveFor(10,inches,70,velocityUnits::pct);
      OdomPID.driveToP(Odom,2,12,1,1);
    }
    // Tank Drivetrain //

    // Left Side Chassis
    if (abs(Controller1.Axis3.position(percentUnits::pct)) > threshold) 
    {
      ChasLfVar = Controller1.Axis3.position(percentUnits::pct);
    } else {
      ChasLfVar = 0;
      LeftDriveSmart.stop();
    }

    // Right Side Chassis
    if (abs(Controller1.Axis2.position(percentUnits::pct)) > threshold) 
    {
      ChasRtVar = Controller1.Axis2.position(percentUnits::pct);
    } else {
      ChasRtVar = 0;
      RightDriveSmart.stop();
    }
    // halfspeed control //
    //Allows speed to be switch to 50 percent for drivetrain
    //Currently button is disabled above
    if (halfspeed == true) {
      LeftDriveSmart.spin(directionType::fwd, ChasLfVar * .50,percentUnits::pct);
      RightDriveSmart.spin(directionType::fwd, ChasRtVar * .50, percentUnits::pct);
    } else {
      LeftDriveSmart.spin(directionType::fwd, ChasLfVar, percentUnits::pct);
      RightDriveSmart.spin(directionType::fwd, ChasRtVar, percentUnits::pct);
    }

  }
  wait(20,msec);
}
int tempTrack(){
  wait(5,sec);
  Odom.trackPosition();
  return 0;
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  task TrackTask(tempTrack);
  //task TrackGoal(purePursuit);
  //task TrackDriveP(MoveTo);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
