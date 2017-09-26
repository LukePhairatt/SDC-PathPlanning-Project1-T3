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


