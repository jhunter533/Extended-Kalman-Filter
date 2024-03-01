#ifndef OdomOOP
#define OdomOOP
//#include "GlobalVariables.h"
#include "Conversion.h"
#include "vex.h"
class Odometry{
    private:
        float prevR; 
        float prevS;

        double FrontLeftPower;
        double BackLeftPower;
        double FrontRightPower;
        double BackRightPower;

        double startAngle;
        double startPosX;
        double startPosY;

        double sDistanceInput;
        double rDistanceInput;
        double sTrackDistance;
        double rTrackDistance;

        double wheelDiameter;

        double absoluteAngle;
        double prevTheta;
        double totalDeltaDistR;
        double yPosGlobal;
        double xPosGlobal;
        double deltaTheta;
        double halfAngle;
        double avgTheta;

        double deltaXGlobal;
        double deltaYGlobal;

        double deltaXLocal;
        double deltaYLocal;
        double r;
        double r2;

        double xTargetLocation;
        double yTargetLocation;

        double xVector;
        double yVector;

        double deltaR;
        double deltaS;
        double xIn,yIn;

    public:
        Odometry() {
            wheelDiameter = 3.25;
            prevR = 0;
            prevS = 0;
            
            prevTheta = absoluteAngle;
            totalDeltaDistR = 0;
            
            deltaTheta = 0;
            halfAngle = deltaTheta/2;
            avgTheta = absoluteAngle-halfAngle;//I'd have to refresh my memory but I think the half angle had something to do with local angle

            deltaXGlobal = 0;
            deltaYGlobal = 0;

            deltaXLocal = 0;
            deltaYLocal = 0;
            r = 0;
            r2 = 0;
            startPosX=0;
            startPosY=0;
            yPosGlobal = startPosY;
            xPosGlobal = startPosX;
            xTargetLocation = xPosGlobal;
            yTargetLocation = yPosGlobal;

            xVector = 0;
            yVector = 0;

            deltaR = 0;
            deltaS = 0;
        

        }
        void draw();
        int trackPosition();
        double getXPosGlobal();
        double getYPosGlobal();
        double getDeltaXG();
        double getDeltaYG();
        double getAbsoluteAngle();
        double getPrevTheta();
        void setXIn(double i);
        void setAng(double a);
        void setYIn(double i);
        void setSDistInput(double i);
        void setRDistInput(double i);

};

//to use Odometry odom;
//task odom.trackposition;
#endif