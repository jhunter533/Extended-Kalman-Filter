
#include "PP.h"

//This function will tell if the robot is intersecting with the target path based off line circle intersect
int purePursuit(Odometry &Odom, PID &PIDOdom){
  double lookDistance=6;
    double xCurr=Odom.getXPosGlobal();
    double yCurr = Odom.getYPosGlobal();
    double lastIndex=0;

    double startIndex=lastIndex;
    for (int i=0; i<path.size();i++){
        double X1=path[i].first;
        double Y1=path[i].second;
        double X2 =path[i+1].first;
        double Y2 = path[i+1].second;

        double localX1= X1-xCurr;
        double localY1 = Y1-yCurr;
        double localX2 = X2-xCurr;
        double localY2= Y2-yCurr;

        bool Intersect = false;

        double dx = localX2-localX1;
        double dy = localY2-localY1;
        double dr = sqrt(dx*dx+dy*dy);
        double D=(localX1*localY2)+(localX2*localY1);
        double Discrim = (lookDistance*lookDistance)*(dr*dr)-(D*D);

        if (Discrim>=0){
            double solx1=((D*dy+sgn(dy)*dx*sqrt(Discrim))/(dr*dr));
            double soly1 =((D*dy+abs(dy)*sqrt(Discrim))/(dr*dr));
            double solx2 =((D*dy-sgn(dy)*dx*sqrt(Discrim))/(dr*dr));
            double soly2 =((D*dy-abs(dy)*sqrt(Discrim))/(dr*dr));

            double minX = std::min(path[i].first,path[i+1].first);
            double maxX= std::max(path[i].first,path[i+1].first);
            double minY = std::min(path[i].second,path[i+1].second);
            double maxY= std::max(path[i].second,path[i+1].second);

            std::pair<double,double> Solution1 = {xCurr+solx1,yCurr+soly1};
            std::pair<double,double> Solution2 = {xCurr+solx2,yCurr+soly2};
            if (((minX<= Solution1.first<=maxX )&& (minY<Solution1.second<=maxY)) ||((path[i].first<= Solution2.first<=path[i+1].first )&& (path[i].second<=Solution2.second<=path[i+1].second))) {
                Intersect=true;
                if (((minX<= Solution1.first<=maxX )&& (minY<=Solution1.second<=maxY)) &((minX<= Solution2.first<=maxX )& (minY<=Solution2.second<=maxY))){
                     //both are in bounds
                    
                    if (sqrt(((path[i+1].first-Solution1.first)*(path[i+1].first-Solution1.first))+((path[i+1].second-Solution1.second)*(path[i+1].second-Solution1.second)))<sqrt(((path[i+1].first-Solution2.first)*(path[i+1].first-Solution2.first))+((path[i+1].second-Solution2.second)*(path[i+1].second-Solution2.second)))) {
                        goalP=Solution1;
                   
                    } else {
                        goalP=Solution2;
                   
                    }
                } else {
                    if (minX<=Solution1.first<=maxX && minY<=Solution1.second<=maxY){
                        goalP=Solution1;
                  
                    } else {
                        goalP = Solution2;
                    }
                    if (sqrt(((path[i+1].first-goalP.first)*(path[i+1].first-goalP.first))+((path[i+1].second-goalP.second)*(path[i+1].second-goalP.second)))<sqrt(((path[i+1].first-xCurr)*(path[i+1].first-xCurr))+((path[i+1].second-yCurr)*(path[i+1].second-yCurr)))){
                        lastIndex = i;
                        break;

                    } else {
                        lastIndex = i+1;
                        
                   }
                }
            } else {
                Intersect = false;
                goalP ={path[lastIndex].first,path[lastIndex].second};
                }
            if (i==path.size()){
              Intersect=false;
              goalP={path[i].first,path[i].second};
            }
            }

      task::sleep(15);
    }
    return 1;
}
//This function creates the desired path.
 void PathGens(Odometry &Odom, PID &PIDOdom){
    double space = 5;
   for (int j =0;j<coord.size()-1;j++){

    double startPX=coord[j].first;
    double startPY=coord[j].second;
    double endPX=coord[j+1].first;
    double endPY=coord[j+1].second;
    double vectorX=endPX-startPX;
    double vectorY=endPY-startPY;

    double magnitude = sqrt(vectorX*vectorX+vectorY*vectorY);
    vectorX/=magnitude;
    vectorY/=magnitude;
    vectorX*=space;
    vectorY*=space;
    double vector[2]={vectorX,vectorY};
    double pFit=floor(magnitude/space);
    for(int i = 0;i<pFit;i++){
    double xPush=startPX+vectorX*i;
    double yPush=startPY+vectorY*i;
    path.push_back(std::make_pair(xPush,yPush));
    //std::cout<< startPX;
    //std::cout <<", ";
    //std::cout <<startPY;
    //std::cout <<", ";
    //std::cout<<pFit;
    //std::cout<<"coord: ";
}}
path.push_back(coord.back());
//coord.push_back(std::make_pair(endPX,endPY));

for (const auto& p : path) {
    std::cout << p.first << ", " << p.second << std::endl;
}
 }

int sgn(double x){
    if (x<0){
        return -1;
    } else {
        return 1;
    }
}
//This path is what powers the robot to move to the target.
int MoveTo(Odometry &Odom, PID &PIDOdom){
  double prevEL = 0;
  double prevER=0;
  double accErrorL=0;
  double accErrorR=0;
  double accR;
  double accL;
  double angleError;
  double tAngle;
  double linError;
  double FLP,FRP,BLP,BRP;
  while(1){
 
    gx= goalP.first-Odom.getXPosGlobal();
    gy=goalP.second-Odom.getYPosGlobal();
    linError=sqrt(gx*gx+gy*gy);
    tAngle=atan2(gy,gx);
    angleError=tAngle-Odom.getAbsoluteAngle();

    FLP=linError-angleError;
    BLP=linError-angleError;
    FRP = linError+angleError;
    BRP=linError+angleError;

    accL=FLP-prevEL;
    accR=FRP-prevER;

    prevEL=FLP;
    prevER=FRP;
    double FFL=kV*FLP+kA*accErrorL;
    double FFR=kV*FRP+kA*accErrorR;

    double FBL=kP*(FLP-FrontLeft.velocity(velocityUnits::rpm));
    double FBR=kP*(FRP-FrontRight.velocity(velocityUnits::rpm));
    
    FrontLeft.spin(forward,FBL+FFL,voltageUnits::volt);
    BackLeft.spin(forward,FBL+FFL,voltageUnits::volt);
    BackRight.spin(forward,FBR+FFR,voltageUnits::volt);
    FrontRight.spin(forward,FBR+FFR,voltageUnits::volt);
    
    task::sleep(15);
}
FrontLeft.stop();
FrontRight.stop();
BackRight.stop();
BackLeft.stop();
  //return 0;
}
//This function finds the distance between points since if you add line smoothing or other things you may lose that info
void DistBP(std::vector<std::pair<double,double>> path, double k, Odometry &Odom, PID &PIDOdom){
  Curvature.resize(path.size());
  Curvature[0]=0;
  Curvature[path.size()]=0;

  for(int i=1;i<path.size();i++){
    iDistx=iDistx+(path[i].first-path[i-1].first);
    iDisty=iDisty+(path[i].second-path[i-1].second);

    iDists[i]=hypot(iDistx,iDisty);

  }
}
//Finds the curvature of each point
//This and velocity function exist so you don't carine into a corner
void Curve(std::vector<std::pair<double,double>> path,double k, Odometry &Odom, PID &PIDOdom){
  for (int j =1;j<path.size()-1;j++){
    double x1=path[j-1].first;
    double x2=path[j].first;
    double x3=path[j+1].first;
    double y1=path[j-1].second;
    double y2=path[j].second;
    double y3=path[j+1].second;
    double k1=.5*((x1*x1)+(y1*y1)-(x2*x2)-(y2*y2))/(x1+.0001-x2);
    //std::cout<<"Curve5:" << Curvature[i];
    double k2=(y1-y2)/(x1+.0001-x2);
    //std::cout<<"Curve6:" << Curvature[i];
    double b=.5*((x2*x2)-2*x2*k1+(y2*y2)-(x3*x3)+2*x3*k1-(y3*y3))/(x3*k2-y3+y2-x2*k2+.0001);
    double a=k1-k2*b;
    double r = sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));
    Curvature[j]=1/r;
    //std::cout<<"Curve:" << Curvature[i];

    double minVcalc=k/Curvature[j];
    //std::cout<<"r";
    double minVel=std::min(maxVel, minVcalc);
    //std::cout<<"t";
    oldVel[j]=minVel;
  }
}
//Finds the target velocity for each point
void Velocity(double a,Odometry &Odom, PID &PIDOdom){
  for (int i =NPoint.size()-1;i>=0;i--){
    double vf=sqrt(oldVel[i+1]*oldVel[i+1]+2*a*iDists[i]);
    oldVel[i]=std::min(oldVel[i],vf);
  }
}