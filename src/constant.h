#ifndef CONSTANT_H
#define CONSTANT_H

// priority levels for costs
const double COLLISION  = 1e6;    // path collision
const double DANGER     = 1e6;    // car in front and behind in lane changing
const double CONGESTION = 1e5;    // choose lighter traffic lane to drive
const double REACH_GOAL = 1e5;    // s,d distance to the goal point (want s big, d small for lane changing)
const double COMFORT    = 1e4;    // lane change toward the goal lane
const double EFFICIENCY = 1e5;    // speed efficiency- we don't want to get stuck behind the slow car for a long time
                                  // switch the lane if we could make it go faster 


// some parameters for the cost function
const double VEHICLE_RADIUS = 2;  // considering collision free zone (used by trajectory check) 
const double STOP_COST = 0.7;     // for velocity cost function
const double BUFFER_V  = 0.5;     // about 1 mph under speed limit (velocity cost function)

// vehicle and trajectory
const double SPEED_LIMIT = 22;    // m/s about 49 mph 
const double MAX_ACCEL   = 10;    // m/s2 about 1g, limited to 1g max no more
const double TOTAL_ACCEL = 10;    // m/s2
const double MAX_JERK    = 50;    // m/s3
const double BUFFER_DIST = 50;    // min m (s distance) time to slow down 
const double BUFFER_TIME = 3;     // min sec time gap between vehciles for changing lane



#endif
