# **Path-Planning-Project-1 Term-3**
Self-Driving Car Engineer Nanodegree Program


[//]: # (Image References)
[image0]: ./result/task.png "task"
[image1]: ./result/spline.png "spline"
[image2]: ./result/cost.png "cost"
   
### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946 m (4.32 miles) highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3 and total jerk that is less than 50 m/s^3.

### The map of the highway (/data/highway_map.txt)
Each waypoint in the list contains  [x,y,s,dx,dy] values (all are in meters) 
x and y are the waypoint's map coordinate position.
s is the distance along the road to get to that waypoint. 
the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Road Configuration and Axis Orientation
The dual highway with 3 lanes on each side.
```sh


                		s 
                		^                Forward
                		|
      (other side traffic)	|    4m     |     4m     |    4m      |
				|           |            |            |
              		  	+----------------------------------------> d
		lane id               0           1             2

                   		    Left         Middle        Right

                                  		  Rear
  x
  ^
  |
  |
  |
  +------> y  
  map coordinate
                              
```

### Main car's localization Data (No Noise) received from the simulator  
["x"] The car's x position in map coordinates (m)  
["y"] The car's y position in map coordinates (m)  
["s"] The car's s position in frenet coordinates (m)  
["d"] The car's d position in frenet coordinates (m)  
["yaw"] The car's yaw angle in the map (degree)  
["speed"] The car's speed (MPH)  
["previous_path_x"] The previous list of x points (processed points removed) previously given to the simulator (m)  
["previous_path_y"] The previous list of y points (processed points removed) previously given to the simulator (m)  
["end_path_s"] The previous list's last point's frenet s value (m)  
["end_path_d"] The previous list's last point's frenet d value (m)  

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)
["sensor_fusion"] A 2d vector of other cars (traffic) in the following format,

[car ID, x position (m), y position (m), x velocity (m/s), y velocity (m/s), s position in frenet (m), d position in frenet (m). 


### Output to the simulator
The simulator is expecting the list of x,y points in the map coordinate for the car to go in the next cycle. As mentioned below, the simulator executes this point every 20 ms loop. Therefore the distance between the points would also reflect the speed of the car. We need to take this into account planning the path.

## System Design 

The project has the following files:

```sh
Files: 
	main.cpp		: Task control
        vehicle.cpp,vehicle.h   : Vehicle state update, Spline line trajectory generator
        traffic.cpp,trafic.h    : Current traffic (other cars) and prediction
        helper.cpp,helper.h     : General functions 
        cost_function.h         : Cost of trajectory
        constant.h              : Parameters
``` 

The functionlities are

```sh
task flow: main.cpp 
      * Car state update/localisation (data from the simulator- every loop) 
      * Traffic update and prediction (data from the simulator- every loop) 
      * Spline line path planning (every loop) 
      * Behaviour control (every 50 loops) e.g. stay in lane, lane change left or right based on the cost

```

### Execution Flow

![task flow][image0]
Figure 1: Task Flow

In the **main.cpp**:
The task runs in sequential order. The car and traffic information(sensor_fusion) are first get updated along with the reference goal point for our vehicle to get to.
[code: lines 144-196]

If the behaviour planning is not activate, the task runs the normal routine of the path planning and motion control via trajectory generation on the current lane (no lane change).  
[code: lines 262-289]

After the cold start (> 100 loops) and every 50 loops cycle, the behaviour control is activated to plan for an appropriate action such as staying in this lane or changing one  
determined by the cost (thid will be presented later on).
[code: lines 199-258]


### Path Planning (Trajectory Generation)     
**Spline line**  
In this project, the vehicle trajectory has been generated with the spline library (spline.h) fitting through constriant points.

For the smooth transition, the last 2 points from the previous path were used as a constriant for a spline line fitting. Together with the next 3-constraint points (experimenting with 30m apart) where we want the vehicle to be in the future (as shown by the figure 2), the full spline model was then generated for the smooth transition.

					[code: vehicle.cpp in SplineLineTrajectory(..), lines 185-220]


![spline line][image1]
Figure 2: Spline line fitting

In this example of changing lane, the vehicle will be in the centre of the new lane at 30 m forward from the last point on the given speed.
					
					[code: vehicle.cpp in SplineLineTrajectory(..), lines 218-220]

It is noted that calculation for the spline line was done in the vehicle frame to make sure the stability of the line fitting.

					[code: vehicle.cpp in SplineLineTrajectory(..), lines 230-242]

Having had the spline model, the new points are then generated from the last point in the prevoius trajectory for 60 meters forward. The path generation in this step is also taking into account the required target speed (more details on this speed are explained later in "collision check"). 

					[code: vehicle.cpp in SplineLineTrajectory(..), lines 251-282]

We also need to convert the new x,y points back to world coordinate. 

					[code: vehicle.cpp in SplineLineTrajectory(..), lines 274-279]


The whole path are then formed by the previous points plus the new trajectory points.

					[code: vehicle.cpp in SplineLineTrajectory(..), lines 286-289]

In addition to this xy path, the sd path transformed from this xy path is required for the cost calculation in behavioral planning.

					[code: helper.cpp in XYtoSD(..), lines 197-215]

**Collision check**   
Futher calculation has been made here to make sure that the new planned points conform the speed and accelleration limit as well as collision with the vehicle in front. In order to avoid collision with the front vehicle in the desire lane, the speed of the vehicle (target speed) was conditioned on the gap between our car and a closest car in front.  


```sh

      v1- m/s                   v2- m/s
      our car                 traffic car            acceptable spot
       +--------------------+    *--------------*        |
                            |<------ Gap ------>|

                            |<-------- Buffer -----------| 

```

In this implementation, if the gap between the end of our car trajectory and the predicted trajectory end of the traffic less than the Buffer limit. Then we  slow down our car to maintain the buffer distance by 0.15 m/s or about 0.75g(dv/dt = 0.15/0.02) where the limit is a little more than 1g. Improvement could be done to take out the speed/breaking in propotion to the gap (but meet the car fesability) so we don't need to unnecessary slow down far too much.

					[code: vehicle.cpp in get_next_target_speed(..), lines 77-150]

If there are no potential collision, this speed is then fed to the next level of control to make sure it comply with the speed/accelleration limit (see below).

**Speed and accelleration control**   
Without using the PID or MPC controller (e.g. perfect controller- no significant noise and delay), the reference speed (target speed) for the path generation has been recorded every loop as a reference for the next cycle. We assume the actual speed won't be too far off. In this case if the current reference speed is below the speed limit, we could speed the car up by 0.15 m/s or slow it down by the same amount if it goes over the limit.  

					[code: vehicle.cpp in get_next_target_speed(..), lines 150-160]



### Behaviour Control and Cost Function 

**Finite State**  
In navigating our car through the road traffic, the behaviour of driving in this work has a state of   
1- keeping lane,   
2- changing lane left,   
3- changing lane right.  

In the design of the state transition, the car will only check for the current lane and its neighbour lane. So if the car is in lane 0 it only has 'keeping lane' or 'changing lane right' transition state. But if the car is in lane 1 it has 3 options, 1-keeping lane, 2-goto the left, 3-goto the right in the transition state. 

**Cost Functions**  
The transition of these states are defined by the 'MINIMUM'cost of actions in association with the traffic, our car state (current lane), the goal point, and the trajectory defined by the following cost factors.

The implementation of these cost function are in **cost_functions.h**.

* toward the goal lane: reward if moving to the goal lane, penalise if moving away  
 (see line 115, **change_lane_cost(..)**)   
* distance from the goal point: ideally we want to switch lane only when we are still quite far away from goal and toward the goal lane  
 (see line 130, **distance_from_goal_lane(..)**)  
* traffic in the lane: choose to go into the low traffic lane (small number of cars in that lane)  
 (see line 151, **traffic_in_lane(..)**)  
* collision: check trajectory between our car and others for collision  
 (see line 195, **collision_cost(..)**)  
* time gap between cars in front and behind: the time gap between the front and rear car to measure safety in the lane change  
 (see line 217, **buffer_cost(..)**)  
* speed efficiency: we ideally want to choose the lane that make the car to go with a faster speed  
 (see line 239, **speed_efficiency(..)**)  

Some illustration of these cost terms are presented in figure 3.

![cost][image2]

Figure 3. Cost function elements


**NOTE** In this work the goal point (in lane 2) is set every 2 km along the path. So when the car passes this goal point (car s > goal s), the new goal is used.

In order to check the cost of these actions, the trajectories are generated by the Path Planning as previoulsy presented. The lowest cost will be chosen for the action. However, we will need to make sure that the action is safe by checking collision state and the time gap in front and behind our car. 

					[code: cost_functions.h in get_mincost_action(..) lines 310-345] 



The decision making in driving is influenced by the weight factors on these cost terms (see **constant.h**).

 
### Experiment and Result
The result video is experimented with the weight factors as in constant.h.
The trajectory is a 50-points path with a 0.02 time step (1 second path). The safety related parameters are 

BUFFER_DIST = 50;    // keep distance (default 50) if 40 the car is going faster but more risk of sudden brake
BUFFER_TIME = 3;     // minimum time gap between vehciles considering for changing lane
                     // 2 secs make it more aggressive in driving with more risk taken 
LANE_CHANGE_S = 30;  // get in lane trajectory (60 m makes the smooth and slow change for comfort but may stay outside of the lane too long) 

The car completes the lap of 4.32 miles in about 6 minutes (5.18 mins for driving 50 mph all the way).



## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every 0.02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a 0.02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last give. For the smooth trajectory in the path planning, the last fews points from previous_path_x, and previous_path_y have been used for this transition.


### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```






