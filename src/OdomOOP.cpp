#include "OdomOOP.h"
//#include "OdomOOP.h"
//#include "vex.h"
//#include "PIDOOP.h"
//#include "Conversion.h"
//I didn't make most of this 
// I got it off github but I can't find it anymore
//The idea is to draw the robot on a grid and demonstrate what the odometry moves with a line
//If this needs adjusted for any reason with coordinates or angle read out the stuff manually before relying on this
//Since I didn't make this there is overcompensating to match what the coords read, so read data by sight before visual
void Odometry::draw() {
    
    int textadjustvalue = 55;
    int rowadjust = 39;
    double fieldscale=1.66548042705;
    //Sets graphical things for our display 
    Brain.Screen.setPenWidth( 1 );
    vex::color redtile = vex::color( 210, 31, 60 );
    vex::color bluetile = vex::color( 14, 77, 146 );
    vex::color graytile = vex::color( 49, 51, 53 );
    Brain.Screen.setFillColor(vex::color( 0, 0, 0 ));
    Brain.Screen.setFont(vex::fontType::mono20);
    Brain.Screen.setPenColor( vex::color( 222, 49, 99 ) );

    //Displays all the field tiles, text of odom values, and a dot symbolizing the robot
    Brain.Screen.printAt(40,20 + textadjustvalue, "X-Pos:%f",xPosGlobal);
    Brain.Screen.setPenColor( vex::color( 191, 10, 48 ) );
    Brain.Screen.printAt(40,50 + textadjustvalue, "Y-Pos:%f",yPosGlobal);
    Brain.Screen.setPenColor( vex::color( 141, 2, 31 ) );
    Brain.Screen.printAt(40,80 + textadjustvalue, "Theta:%f",absoluteAngle);
    Brain.Screen.setPenColor( vex::color( 83, 2, 1 ) );
    Brain.Screen.printAt(40,110 + textadjustvalue, "Angle:%f",TurnGyroSmart.rotation(deg));
    Brain.Screen.setPenColor( vex::color( 255, 255, 255 ) );
    Brain.Screen.setFillColor( graytile );
    Brain.Screen.drawRectangle( 245, 2, 234, 234 );
    Brain.Screen.drawRectangle( 245, 2, 39, 39 );
    Brain.Screen.drawRectangle( 245, 80, 39, 39 );
    Brain.Screen.drawRectangle( 245, 119, 39, 39 );
    Brain.Screen.drawRectangle( 245, 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+rowadjust, 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(2*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(3*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 158, 39, 39 );
    Brain.Screen.drawRectangle( 245+(4*rowadjust), 197, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 2, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 80, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 119, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 197, 39, 39 );
    Brain.Screen.setFillColor( redtile );
    Brain.Screen.drawRectangle( 245, 158, 39, 39 );
    Brain.Screen.drawRectangle( 245, 41, 39, 39 );
    Brain.Screen.setFillColor( bluetile );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 41, 39, 39 );
    Brain.Screen.drawRectangle( 245+(5*rowadjust), 158, 39, 39 );
    Brain.Screen.setPenColor( vex::color( 255,255,255));
    Brain.Screen.setFillColor( vex::color(0,0,0) );
    
    //This draws the robot body for position and arm for angle
    double yfieldvalue = ((-yPosGlobal*M_PI*wheelDiameter)/360*fieldscale)+245-10;
    double xfieldvalue = ((xPosGlobal*M_PI*wheelDiameter)/360*fieldscale)+245;
    Brain.Screen.drawCircle(xfieldvalue, yfieldvalue, 10 );
    Brain.Screen.setPenWidth( 4 );
    //Line angle calculation:
    //x1 and y1 are the robot's coordinates, which in our case is xfieldvalue and yfieldvalue
    //angle is the angle the robot is facing, which in our case is Theta
    //(x1,y1, x1 + line_length*cos(angle),y1 + line_length*sin(angle)) = (x1,y1,x2,y2)
    Brain.Screen.drawLine(xfieldvalue, yfieldvalue, xfieldvalue+cos(absoluteAngle)*15, yfieldvalue+ sin(absoluteAngle) *15);
}
int Odometry::trackPosition(){
    absoluteAngle = ConvertToRadians(-TurnGyroSmart.rotation());
    startAngle = M_PI/2;
    //printf("posx %f",xPosGlobal);
    startPosX=(xIn*360)/(M_PI*wheelDiameter);
    //printf("startx %f\n",startPosX);
    startPosY = (yIn*360)/(M_PI*wheelDiameter);
    yPosGlobal = startPosY;
    xPosGlobal = startPosX;
    sTrackDistance = (sDistanceInput*360)/(M_PI*wheelDiameter);
    rTrackDistance = (rDistanceInput*360)/(M_PI*wheelDiameter);
    while(1){
        //change in encoder value between iterations
        deltaR = (Right.position(deg)-prevR);
        deltaS = (Back.position(deg)- prevS);
        //printf("right pos%f\n",Right.position(deg));

        //previous encoder value
        prevR = Right.position(deg);
        prevS = Back.position(deg);
        
        //total change in encoder over all time
        totalDeltaDistR += deltaR;

        //the current heading of the robot converted to radians
        absoluteAngle=ConvertToRadians(-TurnGyroSmart.rotation(deg));


        //the change in theta between iterations
        deltaTheta = absoluteAngle - prevTheta;
        //previous angle
        prevTheta = absoluteAngle;

        //if the robot's change in theta exists then we moved in 2 axis not just 1
        if ((deltaTheta !=0)) {
            halfAngle = deltaTheta/2;
            r = deltaR/deltaTheta;
            r2 = deltaS/deltaTheta;
            //The change in pose in this frame
            deltaXLocal = 2*sin(halfAngle) *(r2-sTrackDistance);
            deltaYLocal = 2*sin(halfAngle) * (r-rTrackDistance);

        //if the angle didn't change then we only moved in one direction
        } else {
            //This is not likely to happen because drift exists
            deltaXLocal = (deltaS);
            deltaYLocal = (deltaR);
            halfAngle = 0;
        }
        avgTheta=absoluteAngle-halfAngle;
        //the change in theta of the coordinates
        //Converting the local pose change to a global one
        //It's kind of hard to explain but something to do with frames and maybe rotating coordinate axis
        deltaYGlobal = (deltaXLocal) *cos(avgTheta) + (deltaYLocal) * sin(avgTheta);
        deltaXGlobal = (deltaYLocal) * cos(avgTheta) - (deltaXLocal) * sin(avgTheta);

        //adds the change to the original coordinates yielding new coordinates
        xPosGlobal += (deltaXGlobal);
        yPosGlobal += (deltaYGlobal);
        printf("Posx: %f\n", xPosGlobal);
        printf("Posy: %f\n", yPosGlobal);
        //draws coordinate grid on brain
        draw();

        task::sleep(10);
    }
    return 1;
}

double Odometry::getXPosGlobal(){
    return xPosGlobal;
}
double Odometry::getYPosGlobal(){
    return yPosGlobal;
}
double Odometry::getDeltaXG(){
    return deltaXGlobal;
}
double Odometry::getDeltaYG(){
    return deltaYGlobal;
}
double Odometry::getAbsoluteAngle(){
    return absoluteAngle;
}
double Odometry::getPrevTheta(){
    return prevTheta;
}
void Odometry::setXIn(double i){
    xIn=i;
}
void Odometry::setYIn(double i){
    yIn=i;
}
void Odometry::setAng(double a){
    startAngle=a;
}
void Odometry::setSDistInput(double i){
    sDistanceInput=i;
}
void Odometry::setRDistInput(double i){
    rDistanceInput=i;
}

//to use Odometry odom;
//task odom.trackposition;