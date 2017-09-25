#include "helper.h"


void Print2DVector(vector<vector<double>> vec){
	int row = vec.size();
	int col = vec[0].size();
	for(int i=0;i<row;i++){
	  for(int j=0;j<col;j++){
	    std::cout << vec[i][j] << ", ";
	  }	
	  std::cout << std::endl;
	}
}


constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2ms(double v) {return v/2.24;}
double ms2mph(double v) {return 2.24*v;}


double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	// Get the next waypoint index from the current x,y location
	// This is the upper bound waypoint
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    // Get the lower bound waypoint
    // Check the cyclic waypoint condition also
	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

    // Coordinate transform based on the given waypoints
    // These waypoints present the transformation frame
	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x (x_x,x_y)onto n(n_x,n_y)
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;
    // projection distance along d axis
	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;
    // Find the closest waypoint index from the s location
    // This is the lower s-bound
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}
    // Cyclic (the another next waypoint to compute heading)
    // This is the upper s-bound
	int wp2 = (prev_wp+1)%maps_x.size();
	
	// compute heading from the xy waypoint
	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);                   // 'relative distance' from the previous point to current s position
	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);    //  add s element to x world  
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);    //  add s element to y world

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);               // add d element to x world
	double y = seg_y + d*sin(perp_heading);               // add d element to y world

	return {x,y};
}

void WorldToCarTransform(const vector<double>& ptsx,const vector<double>& ptsy, vector<double> offset,\
                         vector<double>& vehicle_ptsx, vector<double>& vehicle_ptsy){
    double ref_x = offset[0];
    double ref_y = offset[1];
    double ref_yaw = offset[2];
    double shift_x;
    double shift_y;
    for(int i=0;i<ptsx.size();i++)
	{
	  shift_x = ptsx[i]-ref_x;
	  shift_y = ptsy[i]-ref_y;
	  vehicle_ptsx.push_back( (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw)) );
	  vehicle_ptsy.push_back( (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw)) );
	}
}

void CarToWorldTransform(const vector<double>& ptsx,const vector<double>& ptsy, vector<double> offset,\
                         vector<double>& world_ptsx, vector<double>& world_ptsy){
    double ref_x = offset[0];
    double ref_y = offset[1];
    double ref_yaw = offset[2];
    double x_ref;
    double y_ref;
    double x_point;
    double y_point;
    
    for(int i=0;i<ptsx.size();i++)
	{
	  // vehicle point	
	  x_ref = ptsx[i];
	  y_ref = ptsy[i];
	  // rotate back to normal gloabal frame 
	  x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
	  y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw)); 
	  // add translation component to the global frame
	  x_point += ref_x;
	  y_point += ref_y;
	  world_ptsx.push_back(x_point);
	  world_ptsy.push_back(y_point);
	  
	}
}


vector<vector<double>> XYtoSD(vector<vector<double>> XY, vector<double> maps_x, vector<double> maps_y){
  // XY vector {{vec x},{vec y}} :: XY[0] := {x,....} XY[1] := {y,....}	
  // SD vector {{vec s},{vec d}} :: SD[0] := {s,....} SD[1] := {d,....}	
  // output length = inputlength - 1 due to lacking of heading info in the last point 
  int length = XY[0].size();
  double heading;
  vector<vector<double>> SD {{},{}}; 
  for(int i=0;i<length-1;i++){
    double x = XY[0][i];
    double y = XY[1][i];
    double dx = XY[0][i+1] - x;
    double dy = XY[1][i+1] - y;
    heading = atan2(dy,dx);
    vector<double> sd = getFrenet(x,y,heading,maps_x,maps_y);
    SD[0].push_back(sd[0]);
    SD[1].push_back(sd[1]); 
  }
  return SD;
}


vector<vector<double>> SDtoXY(vector<vector<double>> SD, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y){
  // XY vector {{vec x},{vec y}} :: XY[0] := {x,....} XY[1] := {y,....}	
  // SD vector {{vec s},{vec d}} :: SD[0] := {s,....} SD[1] := {d,....}	
  int length = SD[0].size();
  vector<vector<double>> XY {{},{}}; 
  for(int i=0;i<length;i++){
    double s = SD[0][i];
    double d = SD[1][i];
    vector<double> sd = getXY(s,d,maps_s,maps_x,maps_y);
    XY[0].push_back(sd[0]);
    XY[1].push_back(sd[1]);
  }
  return XY;
}




/*
double get_next_target_speed(const map<int, traffic_data>& tf_data, double mycar_speed, double mycar_end_s, int lane, int horizon){	
  
  //mycar_speed := current speed m/s
  //lane := to go to lane
  //horizon := lookahead steps (1 step = 0.02 second), previous path size
  //output speed := m/s
  
  
  bool too_close = false;                  					// initial flag for near collision condition
  double target_v = mycar_speed;           					// m/s : current target speed 
  double target_s = mycar_end_s;           					// m : keep distance from this point
  // find reference vehicle
  for(auto KV: tf_data){
	// check car in my lane  
    traffic_data tf_fusion = KV.second;
    float d = tf_fusion.d;
    if( d < (2+4*lane+2) && d > (2+4*lane-2) ){
	  // in my lane/how fast it is going
	  double vx = tf_fusion.vx;
	  double vy = tf_fusion.vy;
	  double check_speed = tf_fusion.v;      				// m/s
	  double check_car_s = tf_fusion.s;                 	// m
	  // looking into the future steps with dt = 0.02
	  check_car_s += ((double)horizon*0.02*check_speed);	// m
	  // if our car predicted in the future close to this vehicle if the future
	  // say with in 30 m we need to adjust speed
	  // also we only check the car in front 
	  if( (check_car_s > target_s) && ((check_car_s - target_s) < BUFFER_DIST) )
	  {
		// making some decision here to slow down
	    // say slow down
		too_close = true;
		target_v = check_speed;           					// m/s : desire velocity (match the car in front)
	  }			
	}
    
  }
  
  // speed and accelleration planning
  if(too_close){
    // slow down/ speed up on this cycle to match the car in front  
	double v_diff = target_v - mycar_speed;
	double dt = (double)horizon*0.02;
	double a = v_diff/dt;
	// limit accelleration
	if(abs(a) > 5.0) a = 5.0*a/abs(a);
	// find target velocity
	double dv = a*0.02; 
	mycar_speed += dv;
	
	//cout << "too close v " << mycar_speed << endl;
  }
  else if(mycar_speed < 22.0){								// 22m/s about 49 mph
    // speed up about 0.5g (dv=0.1, dt=0.02) as limited
	mycar_speed += 0.1;
	//cout << "increase v " << mycar_speed << endl;
  }
  else if(mycar_speed >= 22.0){
	// slow down about 0.5g (dv=0.1, dt=0.02) as limited
	mycar_speed -= 0.1;
	//cout << "decrees v " << mycar_speed << endl;
  }
    		
  return mycar_speed;		
}

vector<vector<double>> SplineLineTrajectory(const Vehicle& vehicle, int lane, const map<int, traffic_data>& tf_data, vector<double>& map_s, vector<double>& map_x, vector<double>& map_y)
{
	// get target speed within maximum accelleration by checking the front vehicle in that lane 
	double target_speed = vehicle.target_speed;
	double car_end_s = vehicle.end_path_s;
	int prev_size = vehicle.previous_path_x.size();
	double ref_vel = get_next_target_speed(tf_data, target_speed, car_end_s, lane, prev_size);          			
	
	//cout << "Trajectory speed for lane: " << lane << " speed: " << ref_vel << endl;
	
		
	// a list of way points in xy
	vector<double> ptsx;
	vector<double> ptsy;

	// reference current x,y,heading states
	double ref_x = vehicle.x;
	double ref_y = vehicle.y;
	double ref_yaw = vehicle.yaw;

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
		ref_x = vehicle.previous_path_x[prev_size-1];
		ref_y = vehicle.previous_path_y[prev_size-1];
		double ref_x_prev = vehicle.previous_path_x[prev_size-2];
		double ref_y_prev = vehicle.previous_path_y[prev_size-2];
		ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);
		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
		
	}

	// create distance m look ahead points space from the last previous point in s-coordinate 
	double car_s = vehicle.end_path_s;
	// get in lane in 60 m ahead 
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
	//cout << "ptsx size: " << ptsx.size() << endl;
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
	
	// output points: {{x..},{y..},{target_velocity}}
	vector<vector<double>> xy_out;
	xy_out.push_back(x_out);
	xy_out.push_back(y_out);
	xy_out.push_back({ref_vel});
	return xy_out;
	
}*/


