/*
#include "GlobalVariables.h"
#include "Odometry.h"
#include "vex.h"
#include "Conversion.h"
#include "PID.h"
#include "Eigen/Dense"

//Global variables
Eigen::Matrix3f stateTransition,xKGPS,pGPS,prevPk;
    double prevVy=0;
    double prevX=0;
    double prevY=0;//need to be assigned as starting
    double prevTheta=absoluteAngle;//rad
    double prevVx=0;
    double angVelocity = 0;
    double deltaT=0;


void predict(){

    stateTransition<<prevX+prevVx*deltaT*cos(prevTheta)-prevVy*deltaT*sin(prevTheta),
                    prevY+prevVx*deltaT*sin(prevTheta)+prevVy*deltaT*cos(prevTheta),
                    prevTheta+ConvertToRadians(angVelocity)*deltaT;
    Eigen::Matrix3f FJacobian;
    FJacobian<<1,0,-prevVx*deltaT*sin(prevTheta)-prevVy*deltaT*cos(prevTheta),
                0,1,prevVx*deltaT*cos(prevTheta)-prevVy*deltaT*sin(prevTheta),
                0,0,1;
    Eigen::Matrix3f Qk;
    Qk<<1,.5,0,
        .5,1,0,
        0,0,1;//covariance stuff idk what these should be
    Eigen::Matrix3f prevPk1;
    prevPk1<<.1,0,0,
             0,.1,0,
             0,0,.1;    //random numbers
    prevPk=FJacobian*prevPk1*FJacobian.transpose()+Qk;
    angVelocity = TurnGyroSmart.gyroRate(zaxis, vex::velocityUnits::dps);
    prevVy=Right.velocity(dps);
    prevVx=Back.velocity(dps);
}
void updateGPS(Eigen::Vector3f stateTransition,Eigen::Matrix3f prevPk){
    Eigen::Vector3f zkGPS;
    zkGPS<<GPS.xPosition(inches)-72,
            GPS.yPosition(inches)-72,
            ConvertToRadians(-GPS.rotation(deg)+90);
    Eigen::Vector3f hkGPS;
    
    Eigen::Vector3f ykGPS;
    ykGPS = zkGPS-stateTransition;

    Eigen::Matrix3f Hk;
    Hk<<1,0,0,
        0,1,0,
        0,0,1;  
    Eigen::Matrix3f RkGPS;
    RkGPS<<1,0,0,
            0,1,0,
            0,0,1;
    Eigen::Matrix3f SkGPS;
    SkGPS=Hk*prevPk*Hk.transpose()+RkGPS;

    Eigen::Matrix3f KkGPS;
    KkGPS=prevPk*Hk.transpose()*SkGPS.inverse();


    xKGPS=stateTransition+KkGPS*ykGPS;
    Eigen::Matrix3f I;
    I<<1,0,0,
        0,1,0,
        0,0,1;
    pGPS=(I-KkGPS*Hk)*prevPk;
}
void updateOdom(Eigen::Matrix3f xKGPS, Eigen::Vector3f stateTransition,Eigen::Matrix3f pGPS){
    Eigen::Vector3f zkOdom;
    zkOdom<<(xPosGlobal+deltaXGlobal)*M_PI*2.75/360,
            (yPosGlobal+deltaYGlobal)*M_PI*2.75/360,
            absoluteAngle;
    Eigen::Vector3f hkOdom;
    
    Eigen::Vector3f ykOdom;
    ykOdom = zkOdom-xKGPS;

    Eigen::Matrix3f Hk;
    Hk<<1,0,0,
        0,1,0,
        0,0,1;  
    Eigen::Matrix3f RkOdom;
    RkOdom<<.088,0,0,
            0,.088,0,
            0,0,ConvertToRadians(.061);
    Eigen::Matrix3f SkOdom;
    SkOdom=Hk*pGPS*Hk.transpose()+RkOdom;

    Eigen::Matrix3f KkOdom;
    KkOdom=pGPS*Hk.transpose()*SkOdom.inverse();

    Eigen::Matrix3f xKOdom;
    xKOdom=xKGPS+KkOdom*ykOdom;
    Eigen::Matrix3f pOdom;
    Eigen::Matrix3f I;
    I<<1,0,0,
        0,1,0,
        0,0,1;
    pOdom=(I-KkOdom*Hk)*pGPS;
}
int EKF(){
    while (1){
        predict();
        updateGPS(stateTransition,prevPk);
        updateOdom(xKGPS,stateTransition,pGPS);
        deltaT+=.010;
        wait(10,msec);
    }
return 0;
}
*/