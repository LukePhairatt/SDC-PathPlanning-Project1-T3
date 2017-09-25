#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <map>
#include <vector>

using namespace std;

struct traffic_data{
    int id   =0;
    int lane =0;
    double x =0;
    double y =0;
    double vx=0;
    double vy=0;
    double v =0;
    double a =0;
    double s =0;
    double d =0;
    double t =0;
}; 


class Traffic {
public:
  
  map<int, traffic_data> data;
  //map<int, vector<sensor_data> > predictions; 
  
  /**
  * Constructor
  */
  Traffic();

  /**
  * Destructor
  */
  virtual ~Traffic();
  
  map<int, vector<traffic_data>> predict(int N= 60, double dt=0.02);
  
  vector<double> get_next_state(double t, int vehicle_id);
  
  
  
};




#endif
