#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include <map>
//-----------------------------//
#include "helper.h"
#include "vehicle.h"
#include "traffic.h"
#include "cost_functions.h"

using namespace std;

// Socket data between Simulator
using json = nlohmann::json;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  // reading reference way points
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  
  // starting lane 
  int lane = 1;
  // local goal point  
  double goal_s = 2000.0;  // 2000 m s-forward
  int loop_count = 0;
  Vehicle myVehicle(lane);
  Traffic traffic;
  

  h.onMessage([&loop_count, &myVehicle, &traffic, &goal_s, &lane,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]\
             (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
            //j[1] is the data JSON object
            //cout << loop_count << endl;
            loop_count++;
            
            // ----------------------------------- //
          	//         Input Data Management       //
          	// ----------------------------------- //
          	
        	// Main car's localization Data
          	double car_x = j[1]["x"];                          	// m
          	double car_y = j[1]["y"];							// m
          	double car_s = j[1]["s"];							// m
          	double car_d = j[1]["d"];							// m
          	double car_yaw = deg2rad(j[1]["yaw"]);              // convert degree to radian
          	double car_speed = mph2ms(j[1]["speed"]);           // m/s         

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];		// m
          	auto previous_path_y = j[1]["previous_path_y"];		// m
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];				// m
          	double end_path_d = j[1]["end_path_d"];				// m

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];			// all in metric units
          	
          	// Get previous path size that reamined
          	int prev_size = previous_path_x.size();     	
			// Define the actual x,y points for the simulator
			vector<double> next_x_vals;
          	vector<double> next_y_vals;
          	              	
          	// start from the previous path points that remained for smooth planning
            // WARNING: if things change all the sudden this path won't get changed 
          	for(int i = 0; i < previous_path_x.size(); i++)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);					
			}
			

            // ----------------------------------- //
          	//         Increment Local Goal        //
          	// ----------------------------------- //
          	if(goal_s < car_s){
          	  goal_s += car_s; 
          	  std::cout << "Setting new local goal s: " << goal_s << std::endl; 
		    }
		    
		    // ----------------------------------- //
		    //         Update Current Traffic      //
		    //            sensor fusion            //
		    // ----------------------------------- //
		    // [myVehicle]
		    myVehicle.lane = lane;
		    myVehicle.x = car_x;
          	myVehicle.y = car_y;
          	myVehicle.s = car_s; 
          	myVehicle.d = car_d;
          	myVehicle.yaw = car_yaw;
          	myVehicle.v = car_speed;
          	myVehicle.previous_path_x = next_x_vals;
          	myVehicle.previous_path_y = next_y_vals;
          	myVehicle.end_path_s = end_path_s;
          	myVehicle.end_path_d = end_path_d;
          	myVehicle.lanes_available = 3;
          	myVehicle.goal_lane = 2;
          	myVehicle.goal_s = goal_s;
          	myVehicle.goal_d = double(2+4*myVehicle.goal_lane);
          	
          	
          	// first loop (set end path to initial for path planning)
          	if(prev_size < 2)
          	{
          	  myVehicle.end_path_s = car_s;
          	  myVehicle.end_path_d = car_d;
          	  myVehicle.target_speed = 0.0;
			}
          	
          	
          	// [traffic] :: sensor_fusion[i] = [id,x,y,vx,vy,s,d]
          	// update current state
          	for(int i = 0; i < sensor_fusion.size(); i++)
			{
				int car_id = sensor_fusion[i][0];
				traffic.data[car_id].id = sensor_fusion[i][0];
				traffic.data[car_id].x  = sensor_fusion[i][1];
				traffic.data[car_id].y  = sensor_fusion[i][2];
				traffic.data[car_id].vx = sensor_fusion[i][3];
				traffic.data[car_id].vy = sensor_fusion[i][4];
				traffic.data[car_id].s  = sensor_fusion[i][5];
				traffic.data[car_id].d  = sensor_fusion[i][6];				
				traffic.data[car_id].v  = sqrt( (traffic.data[car_id].vx * traffic.data[car_id].vx) + \
				                                (traffic.data[car_id].vy * traffic.data[car_id].vy) );                         
			}
		    
		    // ----------------------------------- //
		    //         Behavior planning           //
		    // ----------------------------------- //
		    
		    //             LANE LAYOUT    
		         
            //               Forward
            // 		|	0	|	1	|	2	|
            //    	  left   middle   right
            //                 Rear
         
            // run this task some time after cold start e.g. when everything up to speed 
            // and every 50 cycles after that
            
			if( !(loop_count%50) && (loop_count >= 100) )
			{
				cout << "---- Run Behavioral loop: " << loop_count << endl;
				// Get possible next states from the current vehicle lane 
			    vector<string> states;
		        if(myVehicle.lane == 0){
			      states = myVehicle.MostLeftLane;
			    }
			    else if(myVehicle.lane == myVehicle.lanes_available-1){
			      states = myVehicle.MostRightLane;
			    }
			    else{
			      states = myVehicle.MiddleLane;	
			    }
				
				// {'lane id',{total_cost,collision, time_gap_front,time_gap_back}}
				map<int,vector<double>> costs;    
				                  
				
				// make traffic prediction in s,d coordinate
				map<int, vector<traffic_data>> predictions;
				predictions = traffic.predict(); 
				
				for(int i=0;i<states.size();i++)
				{
				  // get the lane to go to from the current lane, and the request action 
				  int transit_lane = myVehicle.lane_transition(states[i]);
				  cout << "Trajectory for lane " << transit_lane << endl; 	
				  		  
				  // gen trajectory
				  myVehicle.proposed_lane = transit_lane; // we gonna need this in the trajectory gen and cost function
				  Trajectory trj_propose = myVehicle.SplineLineTrajectory(traffic.data,map_waypoints_s,map_waypoints_x,map_waypoints_y);	
				
				  
				  // Compute cost on the trajectory- passing trajectory and traffic prediction and reference velocity 
				  vector<double> cost_with_result = calculate_cost(myVehicle,trj_propose,predictions);
				  
				  //{'lane id',{total_cost,collision, time_gap_front,time_gap_back}}
				  costs[transit_lane] = cost_with_result;
				}	
				
				// find the lowest cost from the available 
				// also check feasability (e.g. no potential collision)	
				// because the lowest cost doesn't mean collision free
				lane = get_mincost_action(lane, BUFFER_TIME, costs);
                cout << " ----- I am going to lane: ----- " << lane << endl;
		    }
		    
      
            /* ----------------------------------- //
		    //         Trajectory Generation       //
		    // ----------------------------------- */
		    
            // Spline line trajectory
            myVehicle.proposed_lane = lane;
            Trajectory trj_new = myVehicle.SplineLineTrajectory(traffic.data,map_waypoints_s,map_waypoints_x,map_waypoints_y);
            // record this speed for the next cycle reference (speed control)
            myVehicle.target_speed = trj_new.set_velocity;
            
           
            // ----------------------------------- //
		    //           Output to Simulator       //
		    // ----------------------------------- //
        
            // Define a path made up of (x,y) points that the car will visit sequentially every 0.02 seconds
          	// NOTE: Speed and Accelleration with be high with a long next x,y point! 
            
          	
          	// output message points
          	json msgJson;
          	msgJson["next_x"] = trj_new.x;
          	msgJson["next_y"] = trj_new.y;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}















































































