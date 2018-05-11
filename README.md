# Path Planning Project
## Author: Sandeep Patil

[path_planning]: ./sample_images/path_planning.jpg "path_planning"
[car_run]: ./sample_images/car_run.png "car_run"

#### Objective of this project is do path planning to complete ride on given track of 3 lanes.
Path planning involves following activities.
* Sensor Fusion: Collect data of objects around car.
* Localization: Locate of car on map. 
* Prediction: Predict where would be other cars in next few seconds.
* Behavior: Decide what actions to take if there are obstacles or if we want to change our path.
* Trajectory: Based on behavior decision and position of other cars, create trajectory to follow.
* Control: Create control commands to actually drive car as per planned trajectory.   

Following diagram provides details. 

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;![path_planning][path_planning]  

The Simulator provides information of Sensor Fusion and localization and it take cares of control part. So in this project the scope of development is to develop Prediction, Behavior and Trajectory modules. 

## Predictions:
As this is highway drive, we need to predict following things for smooth maneuver.  
* If there is car in front of us and we would collide if we do not slow down.  
* If we want to change lane as there is car front of us check if there are cars in adjacent lane and no car is coming from back side which would collide with us.   
* If there is no car in front of us then we can go with max permissible speed.  

To check status of other cars we use sensor fusion data provided by simulator. 
Sensor fusion has following information.   
car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

* We can use car's s position in frenet coordinates to check if car is nearby.
* With car's d position we can derive in which lane that car is.
* With x and y velocity we can check when that car would be after some time (a second).
* If any car is within 50-meter distance then I mark lane of car as blocked. [ make lane_open flag as false].
* If car in front of our car is within 30-meter range then we enable too_close flag as true.
* If car in front of our car is within 15-meter range then we enable too_too_close flag as true. 
* Also we have noted down closest car distances in each lane that would help to if it is safe to change lane.  


## Behavior: 
Based on information collected in prediction module following decision taken in different situations.
1. If there is no car in front of us then we would try to catch up maximum speed by increasing reference speed. 
2. If we are too close to next car then check if adjacent lanes are open and change lane number accordingly. 
3. If there is car is too close (within 30 m) in front of us and there is no adjacent lane free then we would like to slow down bit so we decrease reference speed. If next car's speed is much lower than our car, then reference speed is further reduced. 
4. If we are in extreme lanes (left and right) and if there is free lane on other extreme end then first check if there is car as obstacle in mid lane, if it’s not then change lane number first to mid lane and then to extreme lane.

## Trajectory Generation.
To drive car, we provide trajectory points to simulator. Then controller module in it take cares of driving the car. Simulator moves car from one point to other in 20 mili seconds. So in one second it can cover 50 points. However, frequency of communication between simulator is less than one second so only few points get consumed. 
1. Simulator provides previous planned trajectory points and we add new points according to behavior that we want to follow.
2. First we generate guiding points till 90 meters in front of car and we use *spline* api to generate trajectory points according to guiding points. 
3. Here we use reference speed and next lane number to create guiding points. 

With above planning, car completes more than 9 miles without any violation of speed limit, jerk, lateral acceleration and collision. 

![car_run][car_run] 


## Simulator and installation details.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Basic Build Instructions
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

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
