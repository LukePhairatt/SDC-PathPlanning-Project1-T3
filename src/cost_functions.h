#ifndef COST_FCN_H
#define COST_FCN_H

#include <iostream>
#include "vehicle.h"
#include "constant.h"




//
//* helper functions *//
//

/*
  struct Trajectory{
	vector<double> s;
	vector<double> d;
	vector<double> x;
	vector<double> y;
	double set_velocity=0;
};
*/

double nearest_approach(const Trajectory& trajectory, const vector<traffic_data>& traffic){
  /*
    Find the closest distance between 2 trajectories
    - trajectory := {{s...},{d...}}
    - traffic := {{traffic_data},{}....}
  */
  double closest = 999999;
  // step through points in the vector
  for(int i=0;i<trajectory.s.size();i++){
    double s = trajectory.s[i];
    double d = trajectory.d[i];
    double tf_s = traffic[i].s;
    double tf_d = traffic[i].d;
    //cout << "traffic: " << tf_s << "  " << tf_d << endl;
    double p_dist =  sqrt(pow(tf_s-s,2) +  pow(tf_d-d,2));
    if(p_dist < closest){
	  closest = p_dist;
	}
  }
  return closest;
}

/*
map<int,vector<traffic_data>> filter_traffic(int lane, const map<int,vector<traffic_data>>& traffic_predictions){
  double d_min = double(2+4*lane-2);
  double d_max = double(2+4*lane+2);
  map<int,vector<traffic_data>> filtered_traffic;
  for(auto KV:traffic_predictions){
    // each vehicle state
	vector<traffic_data> check_traffic = KV.second;
	// only check the current location
	double d = check_traffic[0].d;
	int id = check_traffic[0].id;
	if((d_min<d) && (d<d_max)){
	  // vehicle is in this lane
	  filtered_traffic[id] =  check_traffic;
	}
  }
  return filtered_traffic;    	
}*/

vector<double> get_closest_gap_inlane(int lane, double check_s, const map<int,vector<traffic_data>>& traffic_predictions){
  /*
     Given traffic data: find the closest car in front and behind 
                         and return the time gap
  */
  
  double d_min = double(2+4*lane-2);
  double d_max = double(2+4*lane+2);
  double diff_front = 999999;
  double diff_rear  = 999999;
  // loop through all vehicle
  for(auto KV:traffic_predictions){
	// each vehicle state
	vector<traffic_data> check_traffic = KV.second;
	// only check the car in the given lane 
	// only check the first initial point
	double tf_d = check_traffic[0].d;
	double tf_s = check_traffic[0].s;
	double tf_v = check_traffic[0].v;
	tf_v = max(tf_v,1.0);// safeguard 0 div
	// vehicle is in this lane
	if((d_min < tf_d) && (tf_d < d_max)){
	  // time gap in s	
	  double gap = (check_s - tf_s)/tf_v; 
	  // rear vehicle (gap +)
	  if(gap >= 0.0){
	    // update min rear
	    if(abs(gap) < diff_rear) diff_rear = abs(gap);
	  }
	  // front vehicle (gap -)
	  else{
		// update min front 
		if(abs(gap) < diff_front) diff_front = abs(gap);
		   
	  }
	  
	}
	
  }

  vector<double> min_gap{diff_front,diff_rear};
  return min_gap;
  
}


//
//* Cost Function * //
// 
double change_lane_cost(const Vehicle& myVehicle, const Trajectory& trajectory,\
                        const map<int,vector<traffic_data>>& traffic_predictions){
  /*
     Measure action of moving toward or away from a goal lane
  */							
  int current_lane  = myVehicle.lane;         
  int diff_goal = abs(myVehicle.goal_lane - myVehicle.proposed_lane);
  double cost = 0.0;
  // Penalise: moving away from Goal
  if(diff_goal > current_lane)	cost = COMFORT;
  // Encourage: moving toward Goal
  if(diff_goal < current_lane) cost = -COMFORT;						  
  return cost;
}

double distance_from_goal_lane(const Vehicle& myVehicle, const Trajectory& trajectory,\
                               const map<int,vector<traffic_data>>& traffic_predictions){
  
  /*
     Weight action from a distance to a goal lane
     low d, low  cost, high d, high cost (too far away to reach)
     low s, high cost, high s, low  cost (if too close, we don't have time to react)  
     ideally we want to switch lane when s still too far away, but not too close
  */
  
  double traj_end_s = trajectory.s[trajectory.s.size()-1];      // end s location of the last trajectory point (my vehicle)
  double traj_end_d = trajectory.d[trajectory.d.size()-1];      // end d location of the last trajectory point (my vehicle) 
  double dist_s = abs(myVehicle.goal_s - traj_end_s);          // s distance to goal 
  double dist_d = abs(myVehicle.goal_d - traj_end_d);          // d distance to goal
  
  dist_s = max(dist_s,1.0);                                    // safe guard 0 division
  double factor_sd = 1.0 - exp(-dist_d/dist_s);                // cost factor
  double cost = factor_sd * REACH_GOAL;
  return cost;
}

double traffic_in_lane(const Vehicle& myVehicle, const Trajectory& trajectory,\
                       const map<int,vector<traffic_data>>& traffic_predictions)
{
  /*
     Favor a lane with lighter traffic, so it has less traffic to go through
     Only the car FROM the current location - some dist Behind
                  TO   the current location + some dist Horizon
  */
  
  // finding how many vehicle are in the lane
  // lane centre: d = 2+4*lane
  int lane =  myVehicle.proposed_lane;
  double d_min = double(2+4*lane-2);
  double d_max = double(2+4*lane+2);
  double s_min = myVehicle.s - 20.0;
  double s_max = myVehicle.s + 500.0;
  int counter = 0;
  int n_vehicles = traffic_predictions.size();
  for(auto KV:traffic_predictions){
	// each vehicle state
	vector<traffic_data> check_traffic = KV.second;
	// only check the first point, not the whole path
	// we just want to count the number of vehicles
	double d = check_traffic[0].d;
	double s = check_traffic[0].s;
	// if vehicle is in this lane
	// if vehicle in the zone front (e.g. 500 m) and back (50 m)
	if( (d_min<d) && (d<d_max) && (s_min<s) && (s<s_max) ){
	  counter++; 
	}

  }
  //cout << "n car in lane: " << lane << "::" << counter << endl; 
  
  // cost propotional to a number of vehicles in this lane
  double density = 0.0;
  if(n_vehicles > 0) {density = (double)counter/(double)n_vehicles;}
  //cout << "n_vehicle: " << n_vehicles << endl;
  //cout << "density: " << density << endl; 
  return density*CONGESTION;
	
}


double collision_cost(const Vehicle& myVehicle, const Trajectory& trajectory,\
                      const map<int,vector<traffic_data>>& traffic_predictions){
  /*
     Checking if the given trajectory come close to any vehicle
     - Binary cost function
     - Find the nearest distance to any vehicle and check if above or below the threshold
  */
  
  double closest = 999999;
  for(auto KV:traffic_predictions){
	vector<traffic_data> check_traffic = KV.second;
	//cout << "check dist id: " << KV.first << endl;
	double p_dist = nearest_approach(trajectory, check_traffic);
	if(p_dist < closest){
	  closest = p_dist;
	}
  }			
  // radius thresholding
  //cout << "smallest path dist: " << closest << endl;
  return (closest < VEHICLE_RADIUS)? COLLISION:0.0; 					  
}

vector<double> buffer_cost(const Vehicle& myVehicle, const Trajectory& trajectory,\
                      const map<int,vector<traffic_data>>& traffic_predictions){
  /*
     Sometime all lanes has no collision, but we might want to choose the safest lane by checking
     the distance or time gap (in front and behind traffic within the go to lane)
  */
  
  // proposed lane
  int lane =  myVehicle.proposed_lane;
  // time gap front and back
  vector<double> time_gap = get_closest_gap_inlane(lane, myVehicle.s, traffic_predictions);
  //cout << "time gap front: " << time_gap[0] << " time gap back: "  << time_gap[1] << endl;
  
  // buffer cost- the bigger gap the lower cost
  // e.g. front 19 sec, back 1 sec should cost a lot more than front 10 sec, back 10 sec 
  double time_gap_sum = exp(-time_gap[0]) + exp(-time_gap[1]);               
  double cost = DANGER*time_gap_sum;
  vector<double> buff_cost{cost,time_gap[0],time_gap[1]};
  return buff_cost;
  						 						  
}

double speed_efficiency(const Vehicle& myVehicle, const Trajectory& trajectory,\
                      const map<int,vector<traffic_data>>& traffic_predictions){

  /*
    We ideally want to choose the path with higher velocity but not violate the speed limit or stop
    This also use to avoid getting stuck behind the slow car
  */
  
  double cost = 0.0;
  double target_v = trajectory.set_velocity;
  double set_point_v = SPEED_LIMIT-BUFFER_V;
  // mapping cost function [0,1]
  // case: target_v under the set point, 
  if(target_v < set_point_v){
    cost = STOP_COST*( (set_point_v-target_v)/set_point_v );  
  }
  // case: target_v over the set point but under the limit
  else if( (target_v > set_point_v) && (target_v < SPEED_LIMIT) ){
	cost = (target_v-set_point_v)/BUFFER_V;
  }
  // case: violate the speed limit
  else{
	cost = 1.0;
  }
  
  return cost*EFFICIENCY;
}




//
//* Compute Total Cost * //
// 

vector<double> calculate_cost(const Vehicle& myVehicle, const Trajectory& trajectory,\
                      const map<int,vector<traffic_data>>& traffic_predictions){
  double total_cost = 0.0;
  double ch_lane = change_lane_cost(myVehicle, trajectory, traffic_predictions);
  total_cost += ch_lane;
  double tf_lane = traffic_in_lane(myVehicle, trajectory, traffic_predictions);
  total_cost += tf_lane;
  double sd_goal = distance_from_goal_lane(myVehicle, trajectory, traffic_predictions);
  total_cost += sd_goal;
  double col_cost = collision_cost(myVehicle, trajectory, traffic_predictions);
  total_cost += col_cost;
  vector<double> buf_cost = buffer_cost(myVehicle, trajectory, traffic_predictions);
  total_cost += buf_cost[0];
  double speed_cost = speed_efficiency(myVehicle, trajectory, traffic_predictions);
  total_cost += speed_cost;
  
  //cout << "ch_lane: " << ch_lane << endl;
  //cout << "tf_lane: " << tf_lane << endl;
  //cout << "sd_lane: " << sd_goal << endl;
  //cout << "col_chk: " << col_cost << endl;
  //cout << "buf_chk: " << buf_cost[0] << endl;
  //cout << "speed_eff: " << speed_cost << endl;
  //cout << " --- total cost---- " << total_cost << endl;
  
  double time_gap_front = buf_cost[1];
  double time_gap_back  = buf_cost[2];
  
  vector<double> cost_with_result{total_cost,col_cost,time_gap_front,time_gap_back};
  return cost_with_result;
}


//
//* Get Min Cost + Safety Check* //
// 

int get_mincost_action(int current_lane, double min_time_gap, map<int,vector<double>>& costs){
  /*
     Get the lowest cost action e.g. changing lane, keeping lane 
     Also make sure the lowest cost has no potential collision
     costs := {'togo lane id', vector 'cost_with_result'}
     cost_with_result := {total_cost,col_cost,time_gap_front,time_gap_back}
  */
  
  // Get min cost from these actions 
  double min_cost = costs.begin()->second[0];
  int togo_lane = costs.begin()->first;
  for(auto KV: costs){
    double cost = KV.second[0];
    // cost is lower with this one
	if(cost < min_cost)
	{
	  togo_lane = KV.first;
	}  	
  }
  // Check collision and time gap on this selected action 	
  vector<double> cost_data = costs[togo_lane];
  double collision_cost = cost_data[1];
  double time_gap_front = cost_data[2];
  double time_gap_back  = cost_data[3];
  if((collision_cost == 0.0) && (time_gap_front > min_time_gap) && (time_gap_back > min_time_gap))
  {
	// action path is ok
    return togo_lane;
  }
  else
  {
	// action path is not ok: so stay in lane
	return current_lane;
  }
	
}


#endif
