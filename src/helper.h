#ifndef HELPER_H
#define HELPER_H


#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include "spline.h"
#include "vehicle.h"
#include "traffic.h"
#include "constant.h"


using namespace std;


// Print 2D vector
void Print2DVector(vector<vector<double>> vec);

// For converting back and forth between units
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double mph2ms(double v);
double ms2mph(double v);
double distance(double x1, double y1, double x2, double y2);

// sd,xy conversion
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

// XY vector {{vec x},{vec y}} :: XY[0] := {x,....} XY[1] := {y,....}	
// SD vector {{vec s},{vec d}} :: SD[0] := {s,....} SD[1] := {d,....}	
// output length = inputlength - 1 due to lacking of heading info in the last point 
vector<vector<double>> XYtoSD(vector<vector<double>> XY, vector<double> maps_x, vector<double> maps_y);
vector<vector<double>> SDtoXY(vector<vector<double>> SD, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

// Vehicle-World transform
void WorldToCarTransform(const vector<double>& ptsx,const vector<double>& ptsy, vector<double> offset,\
                         vector<double>& vehicle_ptsx, vector<double>& vehicle_ptsy);

void CarToWorldTransform(const vector<double>& ptsx,const vector<double>& ptsy, vector<double> offset,\
                         vector<double>& world_ptsx, vector<double>& world_ptsy);


#endif


