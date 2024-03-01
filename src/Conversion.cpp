#include "Conversion.h"
#include "vex.h"


//Quick custom function to convert to degrees
double ConvertToDeg(double radians) {
  return (radians *180/M_PI);
}
//Quick custom function to convert to radians
double ConvertToRadians(double degree) {
  return (degree*M_PI/180);
}
//Custom function for the odometry movement to check if the angle calculated is quicker from a different direction
//So rather than trying to turn 358 degrees it would turn 2 degrees the other way
/*
double shortAngle(Odometry &Odom, PID &PIDOdom) {
  double angleTarget;
  double Ang1=ConvertToDeg(atan2(PIDOdom.getYVector(),PIDOdom.getXVector()))-ConvertToDeg(Odom.getAbsoluteAngle());
  double fastAngle = Ang1-ConvertToDeg(Odom.getAbsoluteAngle())-360;
  double fastAngle2=Ang1-ConvertToDeg(Odom.getAbsoluteAngle())+360;
  if(fabs(Ang1)<fabs(fastAngle) && fabs(Ang1) <fabs(fastAngle2)){
    angleTarget=ConvertToRadians(Ang1);
          printf("1 \n");
  }
  if(fabs(Ang1)>fabs(fastAngle)&&fabs(fastAngle) < fabs(fastAngle2)){
    angleTarget=ConvertToRadians(fastAngle);
      printf("2 \n");
  }
  if(fabs(Ang1)> fabs(fastAngle2) &&fabs(fastAngle2)<fabs(fastAngle2)){
    angleTarget=ConvertToRadians(fastAngle2);
          printf("3 \n");
  }
  printf("ang1 %.5f\n", Ang1);
  printf("fastang1 %.5f\n", fastAngle);
  printf("fastang2 %.5f\n", fastAngle2);
  return angleTarget;
}
*/