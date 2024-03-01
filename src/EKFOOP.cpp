/*
#include "EKFOOP.h"

void EKF::predict(){
    stateTransition<<prevX+prevVx*deltaT*cos(prevTheta)-prevVy*deltaT*sin(prevTheta),
                    prevY+prevVx*deltaT*sin(prevTheta)+prevVy*deltaT*cos(prevTheta),
                    prevTheta+ConvertToRadians(angVelocity)*deltaT;
    FJacobian<<1,0,-prevVx*deltaT*sin(prevTheta)-prevVy*deltaT*cos(prevTheta),
                0,1,prevVx*deltaT*cos(prevTheta)-prevVy*deltaT*sin(prevTheta),
                0,0,1;
    Qk<<1,.5,0,
        .5,1,0,
        0,0,1;//covariance stuff idk what these should be
    prevPk1<<.1,0,0,
                0,.1,0,
                0,0,.1;    //random numbers
    prevPk=FJacobian*prevPk1*FJacobian.transpose()+Qk;
    angVelocity = TurnGyroSmart.gyroRate(zaxis, vex::velocityUnits::dps);
    prevVy=Right.velocity(dps);
    prevVx=Back.velocity(dps);
}
void EKF::updateGPS(Eigen::Vector3f stateTransition,Eigen::Matrix3f prevPk){
    zkGPS<<GPS.xPosition(inches)-72,
            GPS.yPosition(inches)-72,
            ConvertToRadians(-GPS.rotation(deg)+90);

    ykGPS = zkGPS-stateTransition;
    Hk<<1,0,0,
        0,1,0,
        0,0,1;  
    RkGPS<<1,0,0,
            0,1,0,
            0,0,1;
    SkGPS=Hk*prevPk*Hk.transpose()+RkGPS;
    KkGPS=prevPk*Hk.transpose()*SkGPS.inverse();
    xKGPS=stateTransition+KkGPS*ykGPS;
    I<<1,0,0,
        0,1,0,
        0,0,1;
    pGPS=(I-KkGPS*Hk)*prevPk;
}
void EKF::updateOdom(Eigen::Matrix3f xKGPS, Eigen::Vector3f stateTransition,Eigen::Matrix3f pGPS){
    zkOdom<<(xPosGlobal+deltaXGlobal)*M_PI*2.75/360,
            (yPosGlobal+deltaYGlobal)*M_PI*2.75/360,
            absoluteAngle;
    ykOdom = zkOdom-xKGPS;
    Hk<<1,0,0,
        0,1,0,
        0,0,1;  
    RkOdom<<.088,0,0,
            0,.088,0,
            0,0,ConvertToRadians(.061);
    SkOdom=Hk*pGPS*Hk.transpose()+RkOdom;
    KkOdom=pGPS*Hk.transpose()*SkOdom.inverse();
    xKOdom=xKGPS+KkOdom*ykOdom;
    pOdom=(I-KkOdom*Hk)*pGPS;
}
int EKF::SensorFusion(){
    while (1){
        predict();
        updateGPS(stateTransition,prevPk);
        updateOdom(xKGPS,stateTransition,pGPS);
        prevTheta=absoluteAngle;
        deltaT+=.010;
        wait(10,msec);
    }
return 1;
}
*/