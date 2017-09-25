#include <iostream>
#include <cmath>
#include "vehicle.h"
#include "helper.h"

using namespace std;

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane) {
  this->lane = lane;
}

Vehicle::~Vehicle() {}


map<int, traffic_data> Vehicle::filtered_traffic(const map<int, traffic_data>& tf_data, int lane){
	
  map<int, traffic_data> filtered_list;
  for(auto KV: tf_data){
    traffic_data tf_fusion = KV.second;   
    // only get the car in the given lane   
    double d = tf_fusion.d;
    if( d < (2+4*lane+2) && d > (2+4*lane-2) ){
	  int id = KV.first;
      filtered_list[id] = tf_fusion;  
	}	  
  }
  return filtered_list;
} 

map<int, traffic_data> Vehicle::filtered_traffic_front(const map<int, traffic_data>& tf_data, int lane, double position_s){
	
  map<int, traffic_data> filtered_list;
  for(auto KV: tf_data){
    traffic_data tf_fusion = KV.second;   
    // only get the car in the given lane and in front position_s  
    double d = tf_fusion.d;
    double s = tf_fusion.s;
    if( d < (2+4*lane+2) && d > (2+4*lane-2) ){
		if(s > position_s){
	      int id = KV.first;
          filtered_list[id] = tf_fusion;
	    }  
	}	  
  }
  return filtered_list;
} 


int Vehicle::lane_transition(string request_state){
  int transit_lane = 0;	
  int current_lane = this->lane;
  int delta_lane = 0;
  // find land shift from the request lane
  if(request_state.compare("KL") == 0){
	delta_lane = 0;
  }
  else if(request_state.compare("LCL") == 0){
	delta_lane = -1;
  }
  else if(request_state.compare("LCR") == 0){
	delta_lane = 1;	
  }
  else{
	delta_lane = 0;
	cout << "Request lane change not in the list" << endl;  
  }
  
  // shift to the new lane 
  transit_lane = current_lane + delta_lane;
  return transit_lane;
}


double Vehicle::get_next_target_speed(const map<int, traffic_data>& tf_data, double mycar_speed, double mycar_start_s, \
								      double mycar_end_s, int lane, int horizon){
  /*
    mycar_speed := current speed m/s
    lane := to go to lane
    horizon := lookahead steps (1 step = 0.02 second), previous path size
    output speed := m/s
  */
 
  bool too_close = false;                  					// initial flag for near collision condition
  double target_v = mycar_speed;           					// m/s : current target speed 
  double target_s = mycar_end_s;           					// m : keep distance from this point
  
  // filter lane traffic to obtain only the front cars
  map<int, traffic_data> tf_data_filtered = filtered_traffic(tf_data, lane);
  
  // find the closest reference vehicle (in front) if there are any vehicles
  if(tf_data_filtered.size() > 0)
  {
	// get closest distance + the front car only
	int closest_id = -1;
    double min_gap = 999999;
    for(auto KV: tf_data_filtered){
	  // extract key and value traffic data 
	  int id_key =  KV.first;
      traffic_data tf_fusion = KV.second;   
      // check min gap, front car
      double current_gap = abs(tf_fusion.s - mycar_start_s);
      if( (tf_fusion.s > mycar_start_s) && (current_gap < min_gap) ){
        min_gap = current_gap;
        closest_id = id_key;
	  }	  
    } 
     
    // if found the next closest front- check buffer speed against this vehicle
    if(closest_id >= 0){
      double check_speed = tf_data_filtered[closest_id].v;      	// m/s
	  double check_car_s = tf_data_filtered[closest_id].s;          // m	 
	  
	  // looking into the future steps with dt = 0.02
	  check_car_s += ((double)horizon*0.02*check_speed);	        // m
	  
	  // if our car predicted in the future close to this vehicle 
	  // considering buffer distance 
	  if( (check_car_s > target_s) && ((check_car_s - target_s) < BUFFER_DIST) )
	  {
		// making some decision here to slow down
		too_close = true;
		target_v = check_speed;           					        // desire velocity (match the car in front)
	  }	
    } 
    
    
  }
  
  
  // speed and accelleration planning
  if(too_close){
    /*
    // Approach 1: slow down/ speed up on this cycle to match the car in front  
	double v_diff = target_v - mycar_speed;
	double dt = 0.5;//double dt = (double)horizon*0.02;     // time to reach the target speed
	double a = v_diff/dt;
	// limit accelleration
	if(abs(a) > 10.0) a = 10.0*a/abs(a);
	// find target velocity
	double dv = a*0.02; 
	mycar_speed += dv;
	*/
	// Approach 2: slow down to build up gap // speed down about 0.5g (dv=0.1, dt=0.02) limited 1g
	mycar_speed -= 0.15;
	
	//cout << lane << " lane too close: reduce v " << mycar_speed << " dv" <<  dv << endl;
  }
  else if(mycar_speed < 22.0){								// 22m/s about 49 mph
    // speed up about 0.5g (dv=0.1, dt=0.02) limited 1g
	mycar_speed += 0.15;
	//cout << lane << " lane increase v " << mycar_speed << endl;
  }
  else if(mycar_speed >= 22.0){
	// slow down about 0.5g (dv=0.1, dt=0.02) limited 1g
	mycar_speed -= 0.15;
	//cout << "decrees v " << mycar_speed << endl;
  }
  double output_speed = mycar_speed;  		
  return output_speed;
  
}



Trajectory Vehicle::SplineLineTrajectory(const map<int, traffic_data>& tf_data, \
                                         const vector<double>& map_s, const vector<double>& map_x, const vector<double>& map_y){

  // get target speed within maximum accelleration by checking the front vehicle in that lane 
  int lane = this->proposed_lane;
  double target_speed = this->target_speed;
  double car_end_s = this->end_path_s;
  double car_start_s = this->s;
  int prev_size = this->previous_path_x.size();
  double ref_vel = get_next_target_speed(tf_data, target_speed, car_start_s, car_end_s, lane, prev_size); 
  // a list of way points in xy
  vector<double> ptsx;
  vector<double> ptsy;
  // reference current x,y,heading states
  double ref_x = this->x;
  double ref_y = this->y;
  double ref_yaw = this->yaw;
  
  // first loop, no previous path exist, use the current car information 
  if(prev_size < 2)
  {
    // compute the previous xy location
	double prev_car_x = ref_x - cos(ref_yaw);
	double prev_car_y = ref_y - sin(ref_yaw);
	// xy
	ptsx.push_back(prev_car_x);
	ptsx.push_back(ref_x);
	ptsy.push_back(prev_car_y);
	ptsy.push_back(ref_y);		
		 
  }
  // this is the loop after first, use previously remain path
  else
  {
	// xy- the last 2 points
	ref_x = this->previous_path_x[prev_size-1];
	ref_y = this->previous_path_y[prev_size-1];
	double ref_x_prev = this->previous_path_x[prev_size-2];
	double ref_y_prev = this->previous_path_y[prev_size-2];
	ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
	ptsx.push_back(ref_x_prev);
	ptsx.push_back(ref_x);
	ptsy.push_back(ref_y_prev);
	ptsy.push_back(ref_y);
		
  }
  
  // create distance m look ahead points space from the last previous point in s-coordinate 
  double car_s = this->end_path_s;
  // get in lane in e.g. 60 m ahead 
  vector<double> next_wp0 = getXY(car_s+60,(2+4*lane),map_s,map_x,map_y);
  vector<double> next_wp1 = getXY(car_s+90,(2+4*lane),map_s,map_x,map_y);
  vector<double> next_wp2 = getXY(car_s+120,(2+4*lane),map_s,map_x,map_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  // transform to the local VEHICLE coordinate for spline fitting
  for(int i=0;i<ptsx.size();i++)
  {
	double shift_x = ptsx[i]-ref_x;
	double shift_y = ptsy[i]-ref_y;
	ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
	ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
  }

  // create a spline
  tk::spline spl;
  // set x,y point to the spline
  spl.set_points(ptsx,ptsy);

  // control the speed limit from the section distance of 60m with the number of increment
  // |------------------ 60 m --------------------------|
  // |                   v m/s                          |
  // |  0.02 sec   |	          |          |            |         
  // |     d m     |            |          |            |
  // |             x            x          x            x           x          x add 50-pev_s points 
  double target_x = 60.0;
  double target_y = spl(target_x);
  double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
  double d_step = 0.02*ref_vel;                
  double N = target_dist/d_step;
  double x_add_on = 0;
  vector<double> x_out;
  vector<double> y_out;
  
  // Fill up the rest of the path planner to the buffer limit
  // note: path time = 50 steps * 0.02 =  1 secs.
  //       there will be 50 points in the path
  for(int i = 1;i<=50-prev_size;i++)
  {
	double x_point = x_add_on + (target_x)/N;
	double y_point = spl(x_point);
	// move to the next step
	x_add_on = x_point;
		
	// From Vehicle To World
	double x_ref = x_point;
	double y_ref = y_point;
	
	// rotate back to normal WORLD frame after rotating earlier
	x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
	y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw)); 
	// add translation component start from the last prevous points
	x_point += ref_x;
	y_point += ref_y;
	
	x_out.push_back(x_point);
	y_out.push_back(y_point);
  }
  
  // output points: previous remaining points + new traj 
  vector<double> x_whole = this->previous_path_x;                // copy previous x
  vector<double> y_whole = this->previous_path_y;                // copy previous y
  x_whole.insert( x_whole.end(), x_out.begin(), x_out.end() );   // concat new points x
  y_whole.insert( y_whole.end(), y_out.begin(), y_out.end() );   // concat new points y 
  
  // get s,d from x,y of the whole path 
  vector<vector<double>> XY_whole;
  XY_whole.push_back(x_whole);
  XY_whole.push_back(y_whole);
  // {{s...},{d...}}
  vector<vector<double>> SD_whole = XYtoSD(XY_whole,map_x,map_y);
  // output the new trajectory data
  Trajectory trj_out;
  trj_out.x = x_whole;
  trj_out.y = y_whole;
  trj_out.s = SD_whole[0];
  trj_out.d = SD_whole[1];
  trj_out.set_velocity = ref_vel;

  return trj_out;														
	 
}



