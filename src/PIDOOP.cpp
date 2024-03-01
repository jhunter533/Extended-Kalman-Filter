
#include "PIDOOP.h"
//#include "Conversion.h"
//#include "OdomOOP.h"
#define wheelDiameter 3.25
void PID::setMaxSpeed(double s){
    maxSpeed=s;
}
void PID::setMaxIter(int i){
    maxIter=i;
}
void PID::setTurnThresh(int t){
    turnThreshold=t;
}
void PID::setTurnTolerance(double tol){
    turnToTolerance=tol;
}
void PID::setkI(double I){
    kI=I;
}
void PID::setkD(double D){
    kD=D;
}
void PID::setkP(double P){
    kP=P;
}
void PID::setiBound(double b){
    iBound=b;
}
void PID::setThreshold(double t){
    threshold=t;
}
double PID::getXVector(){
    return xVector;
}
double PID::getYVector(){
    return yVector;
}
double PID::getBRPower(){
    return BackRightPower;
}
double PID::getBLPower(){
    return BackLeftPower;
}
double PID::getFRPower(){
    return FrontRightPower;
}
double PID::getFLPower(){
    return FrontLeftPower;
}
void PID::setMaxSpeedT(double s){maxSpeedT=s;}
void PID::setkIT(double I){kIT=I;}
void PID::setKDT(double D){kDT=D;}
void PID::setkPT(double P){kPT=P;}
void PID::setiBoundT(double b){iBoundT=b;}
void PID::driveToNS(Odometry &Odom) {
//ns and turn to need different pid values in same instance and need to call odom functions from odom object
    error = sqrt(pow(yTargetLocation - Odom.getYPosGlobal(),2)+pow(xTargetLocation-Odom.getXPosGlobal(),2));

    derivative = error - prevError;

    prevError = error;

    if(fabs(error) < iBound && error != 0) {
        integral += error;
    } else {
        integral = 0;
    }


    powerDrive =((error * kP) + (derivative * kD) + (integral *kI));
    if (powerDrive>maxSpeed) {
        powerDrive=maxSpeed;
    } 
}

    //This turn to PID is specifically for the Odometry movement because it relies on vectors instead of a target angle
void PID::turnTo(Odometry &Odom) {
    errorT = ((atan2(yVector,xVector))-Odom.getAbsoluteAngle());

    derivativeT = errorT - prevErrorT;
    prevErrorT = errorT;
//  turnPID.iBound = .2;
  //turnPID.threshold = 0.5; cause its in radians here
    if (fabs(errorT) < iBoundT && errorT != 0) {
        integralT += errorT;
    } else {
        integralT = 0;
    }
    powerDriveT = (errorT * kPT) + (integralT * kIT) + (derivativeT * kDT);
    powerDriveConT=ConvertToDeg(powerDriveT);
    if (powerDriveConT > maxSpeedT) {
        powerDriveConT = maxSpeedT;
    } else if (powerDriveConT < -maxSpeedT) {
        powerDriveConT = -maxSpeedT;
    }
}

void PID::turnPTo(double angleTurn) {
    iter=0;
    // Used for Relative Coordinates. For absolute coordinates, comment out
    // following lines for angleTracker

    // Automated error correction loop
    //Loop runs every 15 miliseconds
    //This function is in degrees internally and externally
    //while the error is less than thresholds and the amount of times in the loop is less than threshold
    // This is to account for when you haven't reached your target but the robot is stuck it may exit the loop
    while (fabs(TurnGyroSmart.rotation(degrees) - angleTurn) > turnToTolerance && iter < maxIter) {
        iter += 1;
        //The error is the target angle - current angle 
        error = angleTurn - TurnGyroSmart.rotation(degrees);
        //The derivative is the error-previous error
        //These are defined as 0 before the loop runs
        derivative = error - prevError;
        //Then change the previous error value to the error before the next loop
        prevError = error;

        // Checking if error passes threshold to build the integral
        //The integral allows you to correct for overshooting so you don't want this to always run
        if (fabs(error) < turnThreshold && error != 0) {
        integral += error;
        } else {
        integral = 0;
        }

        // Voltage to use. PID calculation
        double powerDrive = error * kP + derivative * kD + integral * kI;

        // Capping voltage to max speed
        if (powerDrive > maxSpeed) {
        powerDrive = maxSpeed;
        } else if (powerDrive < -maxSpeed) {
        powerDrive = -maxSpeed;
        }
        // Send voltage to motors
        LeftDriveSmart.spin(forward, powerDrive, voltageUnits::volt);
        RightDriveSmart.spin(forward, -powerDrive, voltageUnits::volt);

        this_thread::sleep_for(10);
        /*
        printf("%.5f",powerDrive);
        printf(" , ");
        printf("%.5f",iter);
        printf(" , ");
        printf("%.5f", angleTurn);
        printf(" , ");
        printf("%.5f", angleTurn-TurnGyroSmart.rotation(degrees));
        printf("\n");
        */
    }

    // Angle achieved, brake robot
    LeftDriveSmart.stop(brake);
    RightDriveSmart.stop(brake);

    // Tuning data, output to screen
    
    turnCount += 1;
    error = angleTurn - TurnGyroSmart.rotation(degrees);
    derivative = error - prevError;
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Turn #: %d", turnCount);
    Controller1.Screen.setCursor(1, 13);
    Controller1.Screen.print("iter: %.0f", iter);
    Controller1.Screen.newLine();
    Controller1.Screen.print("error: %.5f", error);
    Controller1.Screen.newLine();
    Controller1.Screen.print("derivative: %.5f", derivative);
    Controller1.Screen.newLine();
    //printf("%.5f \n", TurnGyroSmart.rotation(degrees));
    
}
void PID::driveToP(Odometry &Odom,double xTarget, double yTarget, double alpha,double beta) {
  //the x and y target converted to degrees traveled
  xTargetLocation = (xTarget*360)/(M_PI*wheelDiameter);
  yTargetLocation = (yTarget*360)/(M_PI*wheelDiameter);

  //the difference between targets - current
  xVector = xTargetLocation - Odom.getXPosGlobal();
  yVector = yTargetLocation - Odom.getYPosGlobal();

  //while loop of the vector needed to travel and the angle left to turn
  //once below threshold should exit
  //shortAngle();
//Threshold is in deg
  while(sqrt(xVector*xVector+yVector*yVector)>26 || (atan2(yVector,xVector)-Odom.getAbsoluteAngle()) >=ConvertToRadians(5) ) {
    xVector = xTargetLocation - Odom.getXPosGlobal();
    yVector = yTargetLocation - Odom.getYPosGlobal();
    //Activate 2 subPIDs
    PID::driveToNS(Odom);
    PID::turnTo(Odom);

    //the drive power for each motor
    //combines the drive and turn using alpha to determine the ratio
    FrontLeftPower = beta*(alpha*(powerDrive) -(1-alpha)*powerDriveCon);
    BackLeftPower = beta*(alpha*powerDrive-(1-alpha)*powerDriveCon);
    FrontRightPower = beta*(alpha*powerDrive + (1-alpha)*powerDriveCon);
    BackRightPower = beta*(alpha*powerDrive +(1-alpha)*powerDriveCon);
   
    /*
    //an attempt at reversing if target is passed

    if ((xTargetLocation<=xPosGlobal-1&&xTargetLocation<=xPosGlobal+1) || (yTargetLocation<yPosGlobal-1 && yTargetLocation<yPosGlobal+1)){
    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);
      if(sqrt(xVector*xVector+yVector*yVector)<=2 ||(atan2(yVector,-xVector)-absoluteAngle-M_PI/2)<=ConvertToDeg(2)){
      break;
      }
    } else if ((xTarget>=xPosGlobal+1 && xTarget>=xPosGlobal-1) || (yTargetLocation>yPosGlobal+1 && yTargetLocation>yPosGlobal-1)) {
    FrontLeft.spin(forward,-FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,-FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,-BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,-BackRightPower,voltageUnits::volt);
        if(sqrt(xVector*xVector+yVector*yVector)<=2 &&((atan2(yVector,-xVector)-absoluteAngle-M_PI/2)<=ConvertToDeg(2))){
      break;
      }
    } else {
    break;
      }
    */

    //sends velocity in volts to each motor to actually drive
    FrontLeft.spin(forward,FrontLeftPower,voltageUnits::volt);
    FrontRight.spin(forward,FrontRightPower,voltageUnits::volt);
    BackLeft.spin(forward,BackLeftPower,voltageUnits::volt);
    BackRight.spin(forward,BackRightPower,voltageUnits::volt);

    //print data to console for checking
    
    printf( "      \n");
    printf( "xPos %.5f\n", Odom.getXPosGlobal());
    printf( "yPos %.5f\n", Odom.getYPosGlobal());
    printf( "sqrt %.5f\n", sqrt(xVector*xVector+yVector*yVector));

    printf( "atan2 %.5f\n", atan2(yVector,xVector)-Odom.getAbsoluteAngle());
    printf( "targethe %.5f\n", atan2(yVector,xVector));
    printf( "ytarget %.5f\n", yTargetLocation);
    printf( "xtarget %.5f\n", xTargetLocation);
    printf( "FLP %.5f\n", FrontLeftPower);
    printf( "currtheta %.5f\n",Odom.getAbsoluteAngle());
    printf( "turn pd %.5f\n", powerDriveCon);
    printf( "drivepid pd %.5f\n", powerDrive);
    printf( "New Iteration00 \n");
    printf( "      \n");

    //double ensurance to exit loop if the target is close enough
    /*
    if (fabs(xTarget-xPosGlobal) <=20 && fabs(yTarget-yPosGlobal) <=20){
      break;

    }*/
    //repeat loop every 15 miliseconds

    if (fabs(xVector) <=15 && fabs(yVector) <= 15 ){//&& atan2(yVector,-xVector)-absoluteAngle-M_PI/2<=ConvertToRadians(4)){
      printf("exit \n");
      break;
    }
    task::sleep(15);
  }

//once while loop is not true stop motors
  FrontLeft.stop();
  FrontRight.stop();
  BackRight.stop();
  BackLeft.stop();
  
}