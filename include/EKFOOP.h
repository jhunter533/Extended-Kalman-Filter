#ifndef EKFOOP
#define EKFOOP

#include "vex.h"
#include "Conversion.h"
#include "Eigen/Dense"

class EKF{
    private:
        Eigen::Matrix3f stateTransition,xKGPS,pGPS,prevPk;
        double prevVy;
        double prevX;
        double prevY;//need to be assigned as starting
        double prevTheta;//rad
        double prevVx;
        double angVelocity;
        double deltaT;
        Eigen::Matrix3f FJacobian;
        Eigen::Matrix3f Qk;
        Eigen::Matrix3f prevPk1;
        Eigen::Vector3f zkGPS;
        Eigen::Vector3f hkGPS;
        Eigen::Matrix3f Hk;
        Eigen::Vector3f ykGPS;
        Eigen::Matrix3f RkGPS;
        Eigen::Matrix3f KkGPS;
        Eigen::Matrix3f SkGPS;
        Eigen::Matrix3f I;
        Eigen::Vector3f zkOdom;
        Eigen::Vector3f hkOdom;
        Eigen::Matrix3f RkOdom;
        Eigen::Matrix3f SkOdom;
        Eigen::Vector3f ykOdom;
        Eigen::Matrix3f KkOdom;
        Eigen::Matrix3f xKOdom;
        Eigen::Matrix3f pOdom;
        public:
            EKF(){
                I<<1,0,0,
                   0,1,0,
                   0,0,1;
                prevVy=0;
                prevX=0;
                prevY=0;//need to be assigned as starting
                prevTheta=absoluteAngle;//rad//needs to be getter
                prevVx=0;
                angVelocity = 0;
                deltaT=0;
            }
            void predict();
            void updateGPS(Eigen::Vector3f stateTransition,Eigen::Matrix3f prevPk);
            void updateOdom(Eigen::Matrix3f xKGPS, Eigen::Vector3f stateTransition,Eigen::Matrix3f pGPS);
            int SensorFusion();
     
};
#endif
