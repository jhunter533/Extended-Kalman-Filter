#ifndef PP
#define PP
#include <cmath>
#include <utility>
#include <vector>
#include <iostream>
#include "PIDOOP.h"
#include "OdomOOP.h"
#include "vex.h"
#include "Conversion.h"

#define wheelDiameter 4
#define gearRatio 1
#define RPM 200

double maxVel=wheelDiameter*M_PI*gearRatio*RPM/60;
std::vector<std::pair<double,double>> coord={{10,0},{90,0},{150,30}};
std::vector<std::pair<double,double>> NPoint;
std::vector<std::pair<double,double>> SNewPath;
double iDistx;
double iDisty;
std::vector<double> iDists;
std::vector<std::pair<double, double>>iDist;
std::vector<double> Curvature;
std::vector<double> oldVel;
std::pair<double,double> goalP={};
int purePursuit(Odometry &Odom, PID &PIDOdom);
std::vector<std::pair<double,double>>  coord={{10,0},{90,0},{150,30}};
std::vector<std::pair<double,double>> path;
std::vector<std::pair<double,double>> SNewPath;
 void PathGens(Odometry &Odom, PID &PIDOdom);
 double kV = 0.;
double kA = 0.;
double kP = 0.;
double gx,gy;
int sgn(double x);
int MoveTo(Odometry &Odom, PID &PIDOdom);
void DistBP(std::vector<std::pair<double,double>> path, double k,Odometry &Odom, PID &PIDOdom);
void Curve(std::vector<std::pair<double,double>> path,double k,Odometry &Odom, PID &PIDOdom);
void Velocity(double a,Odometry &Odom, PID &PIDOdom);
#endif