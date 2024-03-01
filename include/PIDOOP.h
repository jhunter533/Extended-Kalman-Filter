#ifndef PIDOOP
#define PIDOOP
#include "Conversion.h"
#include "vex.h"
#include "OdomOOP.h"
class PID{
    private:
        int turnCount;
        int angleTracker;
        double turnToTolerance;
        int maxIter;
        double integral;
        double derivative;
        double error;
        double prevError;
        double powerDrive;
        double kP;
        double kD;
        double kI;
        double threshold;
        double iBound;
        double maxSpeed;
        double powerDriveCon;

        double iter;

        int turnThreshold;

        
       
        double integralT;
        double derivativeT;
        double errorT;
        double prevErrorT;
        double powerDriveT;
        double kPT;
        double kDT;
        double kIT;
        
        double iBoundT;
        double maxSpeedT;
        double powerDriveConT;
        double FrontLeftPower;
        double FrontRightPower;
        double BackRightPower;
        double BackLeftPower;
        double xVector;
        double yVector;
        double xTargetLocation;
        double yTargetLocation;
        
    public:
        PID(){
            //  Distance to target in degrees
            error = 0;
            //  Error degree from the last iteration
            prevError = 0;
            // Derivative of the error. The slope between iterations
            derivative = 0;
            // Accumulated error after threashold. Summing error
            integral = 0;
            // Iterations of the loop. Counter used to exit loop if not converging
            iter = 0;

            turnCount = 0;
            // Relative degree tracking
            angleTracker = 0;
            errorT,prevErrorT,derivativeT,integralT=0;
        }
        void setMaxSpeed(double s);
        void setMaxSpeedT(double s);
        void setkIT(double I);
        void setKDT(double D);
        void setkPT(double P);
        void setiBoundT(double b);
        
        void setMaxIter(int i);
        void setTurnThresh(int t);
        void setTurnTolerance(double tol);
        void setkI(double I);
        void setkD(double D);
        void setkP(double P);
        void setiBound(double b);
        void setThreshold(double t);
        void driveToNS(Odometry &Odom);
            //This turn to PID is specifically for the Odometry movement because it relies on vectors instead of a target angle
        void turnTo(Odometry &Odom);

        void turnPTo(double angleTurn);
        void driveToP(Odometry &Odom, double xTarget, double yTarget, double alpha,double beta);
        double getYVector();
        double getXVector();
        double getBRPower();
        double getBLPower();
        double getFRPower();
        double getFLPower();
};
#endif 