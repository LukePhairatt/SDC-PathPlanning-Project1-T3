#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include "traffic.h"
#include "constant.h"



using namespace std;

struct Trajectory{
	vector<double> s;
	vector<double> d;
	vector<double> x;
	vector<double> y;
	double set_velocity=0;
};

class Vehicle {
public:
  //double keep_distance_buffer;
  // current state
  int lane;
  double s;
  double d;
  double x;
  double y;
  double yaw;
  double v;
  double a;
  double target_speed;
  int lanes_available;
  double max_acceleration;
  int goal_lane;
  int proposed_lane;
  double goal_s;
  double goal_d;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  vector<double> previous_path_s;
  vector<double> previous_path_d;
  double end_path_s;
  double end_path_d;
  // state logic
  string state;
  const vector<string> MostLeftLane {"KL","LCR"};     // Note: can't go left
  const vector<string> MostRightLane {"KL","LCL"};    // Note: can't go right
  const vector<string> MiddleLane {"KL","LCL","LCR"}; // Note: can go left/right
  // trajectory planner
  map<int, vector<Trajectory>> PlanTrajectory;
  
  
  /**
  * Constructor
  */
  Vehicle(int lane);

  /**
  * Destructor
  */
  virtual ~Vehicle();
  
  // get the traffic data within the given lane
  map<int, traffic_data> filtered_traffic(const map<int, traffic_data>& tf_data, int lane);
  map<int, traffic_data> filtered_traffic_front(const map<int, traffic_data>& tf_data, int lane, double s);
  
  // to go to lane based on the current lane and requested action
  int lane_transition(string request_state);
  
  // predict the target speed based on the traffic information
  double get_next_target_speed(const map<int, traffic_data>& tf_data, double mycar_speed, double mycar_start_s, double mycar_end_s, int lane, int horizon);
  
  // trajectory generator
  Trajectory SplineLineTrajectory(const map<int, traffic_data>& tf_data, const vector<double>& map_s, const vector<double>& map_x, const vector<double>& map_y);
  

};





#endif
