# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Path Planning Algorithm
The path planning algorithm is comprised of two main parts: (a): prediction and behavior planning, (b): trajectory generation, which will be described in details in the following.

### Prediction and Behavior Planning
In the prediction part, we receive information about other cars on the road and the traffic around us from ```sensor_fusion``` object. ```sensor_fusion``` object contains information such as ID, and coordinates of each car around us. Based on this information, we decide what to do next (behavior planning) and generate the desired trajectory accordingly. The outputs of the behavior planning are the objective lane and the reference speed which will be used later to generate the trajectory.

The code first loops through all the cars around us whose information is contained in ```sensor_fusion``` (lines 328-344). If there is a car in our lane (lines 332-333), in front of us and within a distance of 30 m (line 342), it sets the variable ```too_close``` to ```true``` (line 344) and attempts changing the lane to pass the car by calling function ```changeLane()``` (line 345).


Function  ```changeLane()``` (defined in lines 197-219) selects the objective lane according to the following logic:
* If we are in the middle lane:
  * Check if the left lane is clear first by calling function ```isLaneFree()```. If the left lane is clear, move there.
  * If the left lane is not clear, check the right lane and move there, if it is clear.

* If we are in the left or the right lanes, check the middle lane and move there, if it is clear.
* If no clear lane is found, the lane won't change and the car will stay in its current lane.


In order to find out if a specified input lane is clear and it's safe to move to that lane, the code uses function ```checkLane()``` (defined in lines 170-194). Inside ```checkLane()```, the code checks the position of all cars around us provided by ```sensor_fusion```. If there is a car in the specified lane, the code checks if its ```s``` value is within a specified range defined by the input vector ```gap```. If the car is within this range, the variable ```is_free``` is set to ```false``` and returned by the function indicating that the specified lane is not clear.

Finally, if the Boolean variable ```too_close``` is ```true```, we reduce the speed gradually in each cycle to avoid colluding the car in front of us (lines 350-353). Additionally, lines 355-358 in the code will ensure that our car keeps its speed as close as possible to a value slightly below the speed limit (50 mph here).     


### Trajectory Generation

In this section (lines 361-466), the code will generate the trajectory which the car should follow based on the car state, past path points and the reference speed and objective lane calculated by the behavior planning module.

In order to generate a smooth trajectory, a spline is first fit to five anchor points ```ptsx, ptsy``` (lines 361-411). These five anchor points include the last two points of the previous trajectory (lines 383-398) or the current position and the calculated previous position, if there is no previous trajectory (lines 371-382), along with three points at distances of 30, 60 and 90 m (lines 400-411). To make the spline calculation simple, the coordinates are transformed (shift and rotation) to local car coordinates (lines 413-420). The spline is fit to these five anchor points at lines 422-426.

In order to ensure a continuous and smooth transition between cycles, the past trajectory points remained in the buffer (the car has not passed these points yet) are copied into the beginning of the new trajectory (lines 432-436). The rest of the points are then generated by evaluating the spline (lines 436-454) and transforming the coordinates to the original global coordinates (456-461).


## Dependencies

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
