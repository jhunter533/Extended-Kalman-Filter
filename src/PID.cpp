/*
#include "PID.h"
#include "GlobalVariables.h"
#include "vex.h"
#include"Conversion.h"

//Constants only maxIter may change but usually it can remain at 300-400
int turnCount = 0;
int angleTracker = 0;
double turnToTolerance = .5;
int maxIter = 700;

//Create PID's from struct
PID_ turnPID;
PID_ drivePID;
PID_ driveX;
PID_ driveY;

//This was originally a drive function for no Strafing thus the name
//This PID uses the magnitude of the vector of the target position as error
void driveToNS() {
  drivePID.iBound = 9;
  drivePID.threshold = 0.5;

  drivePID.kI = 0.002;

  drivePID.kD = 0.001;

  drivePID.kP = 0.035;

  drivePID.error = sqrt((yTargetLocation - yPosGlobal)*(yTargetLocation-yPosGlobal)+(xTargetLocation-xPosGlobal)*(xTargetLocation-xPosGlobal));

  drivePID.derivative = drivePID.error - drivePID.prevError;

  drivePID.prevError = drivePID.error;

  if(fabs(drivePID.error) < drivePID.iBound && drivePID.error != 0) {
    drivePID.integral += drivePID.error;
  } else {
    drivePID.integral = 0;
  }


  drivePID.powerDrive =((drivePID.error * drivePID.kP) + (drivePID.derivative * drivePID.kD) + (drivePID.integral *drivePID.kI));
  if (drivePID.powerDrive>drivePID.maxSpeed) {
    drivePID.powerDrive=drivePID.maxSpeed;
  } 
}

//The x and Y separate functions are artifacts, though if your robot drifts a lot and you only are driving in each direction it can help there
//They don't take into account any other info beside the stated direction
//The tuning of these is the same as the normal one
void driveToX() {
  driveX.iBound = 9;
  driveX.threshold = 0.5;

  driveX.kI = 0.002;

  driveX.kD = 0.001;

  driveX.kP = 0.035;

  driveX.error = xTargetLocation-xPosGlobal;

  driveX.derivative = driveX.error - driveX.prevError;

  driveX.prevError = driveX.error;

  if(fabs(driveX.error) < driveX.iBound && driveX.error != 0) {
    driveX.integral += driveX.error;
  } else {
    driveX.integral = 0;
  }


  driveX.powerDrive =((driveX.error * driveX.kP) + (driveX.derivative * driveX.kD) + (driveX.integral *driveX.kI));
  if (driveX.powerDrive>driveX.maxSpeed) {
    driveX.powerDrive=driveX.maxSpeed;
  } 
}
void driveToY() {
  driveY.iBound = 9;
  driveY.threshold = 0.5;

  driveY.kI = 0.002;

  driveY.kD = 0.001;

  driveY.kP = 0.035;

  driveY.error = yTargetLocation-yPosGlobal;

  driveY.derivative = driveY.error - driveY.prevError;

  driveY.prevError = driveY.error;

  if(fabs(driveY.error) < driveY.iBound && driveY.error != 0) {
    driveY.integral += driveY.error;
  } else {
    driveY.integral = 0;
  }


  driveY.powerDrive =((driveY.error * driveY.kP) + (driveY.derivative * driveY.kD) + (driveY.integral *driveY.kI));
  if (driveY.powerDrive>driveY.maxSpeed) {
    driveY.powerDrive=driveY.maxSpeed;
  } 
}

//This turn to PID is specifically for the Odometry movement because it relies on vectors instead of a target angle
void turnTo() {

  turnPID.iBound = .2;
  turnPID.threshold = 0.5;
  
  turnPID.kP = 0.19;

  turnPID.kI = 0.0025;

  turnPID.kD = 0.9;

  turnPID.error = ((atan2(yVector,xVector))-absoluteAngle);

  turnPID.derivative = turnPID.error - turnPID.prevError;
  turnPID.prevError = turnPID.error;

  if (fabs(turnPID.error) < turnPID.iBound && turnPID.error != 0) {
    turnPID.integral += turnPID.error;
  } else {
    turnPID.integral = 0;
  }
  turnPID.powerDrive = (turnPID.error * turnPID.kP) + (turnPID.integral * turnPID.kI) + (turnPID.derivative * turnPID.kD);
  turnPID.powerDriveCon=ConvertToDeg(turnPID.powerDrive);
  if (turnPID.powerDriveCon > turnPID.maxSpeed) {
    turnPID.powerDriveCon = turnPID.maxSpeed;
  } else if (turnPID.powerDriveCon < -turnPID.maxSpeed) {
    turnPID.powerDriveCon = -turnPID.maxSpeed;
  }
}
*/