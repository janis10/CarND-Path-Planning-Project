# CarND-Path-Planning-Project
**Disclaimer:** The simulation environment in this repository is derived from the [Path planning project of Udacity](https://github.com/udacity/CarND-Path-Planning-Project). If you like it feel free to support it in the above link! 

### Description
We guide a car to safely navigate in a highway with other traffic that is driving `+-10 MPH` of the `50 MPH` speed limit. The simulator provides the car's localization and sensor fusion data Moreover, we have access to a sparse map list of waypoints around the highway. The car should try to go as close as possible to the `50 MPH` speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over `10 m/s^2` and jerk that is greater than `10 m/s^3`.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values, where `x` and `y` are the waypoint's map coordinate position, `s` is the distance along the road to get to that waypoint in meters, `dx` and `dy` define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet `s` value, distance along the road, goes from `0` to `6945.554`.

#### Data provided from the Simulator to the C++ Program

###### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates.

["y"] The car's y position in map coordinates.

["s"] The car's s position in frenet coordinates.

["d"] The car's d position in frenet coordinates.

["yaw"] The car's yaw angle in the map.

["speed"] The car's speed in MPH. 

###### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of `x` points previously given to the simulator. 

["previous_path_y"] The previous list of `y` points previously given to the simulator. 

###### Previous path's end `s` and `d` values 

["end_path_s"] The previous list's last point's frenet `s` value. 

["end_path_d"] The previous list's last point's frenet `d` value. 

###### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [`unique ID`, `x` position in map coordinates, `y` position in map coordinates, `x` velocity in m/s, `y` velocity in m/s, `s` position in frenet coordinates, `d` position in frenet coordinates]. 

## Tips

1. The car uses a perfect controller and will visit every `(x,y)` point it receives in the list every 0.02 seconds. The units for the `(x,y)` points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk. The `(x,y)` point paths that the planner receives should not have a total acceleration that goes over `10 m/s^2`, also the jerk should not go over `50 m/s^3`. 

2. There is some latency between the simulator running and the path planner returning a path. During this delay the simulator will continue using points that it was last given. It's a good idea to store the last points you have used so you can have a smooth transition. Vars `previous_path_x`, and `previous_path_y` show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

3. A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.


### Dependencies
1. Udacity's [Term 3 Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2). 
2. [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), which can be installed using the provided bash scripts by Udacity for either Linux (`install-ubuntu.sh`) or Mac (`install-mac.sh`) systems. 
3. [Eigen](https://eigen.tuxfamily.org) template library for linear algebra.
4. [cmake](https://cmake.org/install/) >= 3.5
5. make >= 4.1 (Linux, Mac)
6. gcc/g++ >= 5.4


### Build and run
Once the dependencies are installed, the main program is built and run as follows:
```
mkdir build
cd build
cmake ..
make
./path-planning
```
For more details on how `main.cpp` uses uWebSocketIO to communicate with the simulator see the [original Udacity repository](https://github.com/udacity/CarND-Path-Planning-Project).