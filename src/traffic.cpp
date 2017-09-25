#include "traffic.h"



/**
 * Initializes Vehicle
 */
Traffic::Traffic() {}

Traffic::~Traffic() {}

// update vehicle location in the next T seconds 
map<int, vector<traffic_data>> Traffic::predict(int N, double dt){
  // N := number of points (more than the generated path)
  // dt = time step (0.02 sec as used by the simulator)   
  double t = 0;             // start time
  map<int, vector<traffic_data>> predictions;  // return predictions
  for(int i=0;i<N;i++)
  {
	  // this applies to all vehicles in 'data' (current state)
	  // start from the current state
	  for(int n = 0; n < this->data.size(); n++)
	  {
		int v_id = data[n].id;
		vector<double> sdv = get_next_state(t,v_id);
		traffic_data tf_data;
        tf_data.id = v_id;
		tf_data.s = sdv[0];
		tf_data.d = sdv[1];
		tf_data.v = sdv[2];
		predictions[v_id].push_back(tf_data);
      }
      // next time step
      t += dt;
  }
  
  return predictions;

}

// Basic next state prediction without lane change (no change in d)
vector<double> Traffic::get_next_state(double t, int vehicle_id){		
  double s = this->data[vehicle_id].s + this->data[vehicle_id].v * t + this->data[vehicle_id].a * t * t/2.0;
  double v = this->data[vehicle_id].v + this->data[vehicle_id].a * t;
  double d = this->data[vehicle_id].d;
  return {s,d,v};
}



