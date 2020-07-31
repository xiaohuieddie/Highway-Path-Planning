# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

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

## Model Documentation
### Prediction
There are five boolean variables: `car_ahead`, `car_left`, `car_righ`, `too_close` and `too_far`. The definition for them are as follows:

 - `car_ahead`: To detect if there is a car in front of ego vehicle.
 - `car_left`: To detect if there is a car on the left side of ego vehicle.
 - `car_righ`: To detect if there is a car on the right side of ego vehicle.
 - `too_close`: To detect if the front car is too close to the ego vehicle.
 - `too_far`: To detect if there is no vehicle in front of the vehicle within 200 meters.

          bool car_ahead = false;
          bool car_left = false;
          bool car_righ = false;
          bool too_close = false;
          bool too_far = true;
          
The code below is looping through the sensor fusion data. Using the Frenet `s` and `d` coordinates, we can easily know which lane the non-ego vehicle is at and how far the non-ego vehicles are away from the ego vehicle. Here we are assuming the width of each is 4 meters and the leftmost lane is `car_lane = 0`. 

To estimate the `s` value of the non-ego vehicle, we are using `check_car_s += ((double)prev_size*0.02*check_speed)` for estimation after executing the previous generated trajectory.

If the car in front of the ego vehicle is less than 30 meters away from the ego vehicle, the `car_ahead` will be turned to `True`. If it is too close and less than 15 meters away, the `too_close` will be triggered to decelerate more in order to avoid collision. In addition, if the distance between the host vehicle and the car in front of it is larger 200 meters, then `too_far` will be set as `true` and causes more acceleration. If the non-ego vehicles which are either left or right hand side of the ego vehicle are 30 meters away(either ahead or behind) from the ego vehicle with respect to `s` distance, the `car_left` and `car_right` will be set to `true` accordingly.
          

          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            float d = sensor_fusion[i][6];
            int car_lane = -1;
            // is it on the same lane we are
            if ( d > 0 && d < 4 ) {
              car_lane = 0;
            } else if ( d > 4 && d < 8 ) {
              car_lane = 1;
            } else if ( d > 8 && d < 12 ) {
              car_lane = 2;
            }
            if (car_lane < 0) {
              continue;
            }
            // Find car speed.
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            // Estimate car s position after executing previous trajectory.
            check_car_s += ((double)prev_size*0.02*check_speed);

            if ( car_lane == lane ) {
              // Car in our lane.
              car_ahead |= check_car_s > car_s && check_car_s - car_s < 30;
              too_close |= check_car_s > car_s && check_car_s - car_s < 15;
              if ((check_car_s > car_s) && ((check_car_s - car_s) < 200)){
                too_far = false; 
              }
            } else if ( car_lane - lane == -1 ) {
              // Car left
              car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            } else if ( car_lane - lane == 1 ) {
              // Car right
              car_righ |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
            }
          }

### Behavior

Based on the prediction of the situation we are in, this code increases the speed, decrease speed, or make a lane change when it is safe. This part decides what to do:
-   If we have a car in front of us, do we change lanes?
-   Do we speed up or slow down?

              if ( car_ahead ) { // Car ahead
            if ( !car_left && lane > 0 ) {
              // if there is no car left and there is a left lane.
              lane--; // Change lane left.
            } else if ( !car_righ && lane != 2 ){
              // if there is no car right and there is a right lane.
              lane++; // Change lane right.
            } else {
              if (too_close){
                ref_vel -= 0.4; 
              } else{
                ref_vel -= 0.224;
              }
            }
          } else {
            if ( lane != 1 ) { // if we are not on the center lane.
              if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) {
                lane = 1; // Back to center.
              }
            }
            if ( ref_vel < 49.5) {
              if (too_far){
                ref_vel += 0.5;
              } else{
                ref_vel += 0.224;
              }
            }
          }

### Trajectory
This code does the calculation of the trajectory based on the speed and lane output from the behavior, car coordinates and past path points.

First, the last two points of the previous trajectory (or the car position if there are no previous trajectory, codes below are used in conjunction three points at a far distance to initialize the spline calculation .

    // if previous size is almost empty when we are starting the vehicle, use the car as the staring reference
          if (prev_size < 2){
            // Use two points that make the path tangent to the current car position
            double prev_car_x = car_x - 1 * cos(car_yaw);
            double prev_car_y = car_y - 1 * sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }

    // Else use the previous path end point as the starting reference
          else{
            //redefine the reference point as previous path end point
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double prev_ref_x = previous_path_x[prev_size - 2];
            double prev_ref_y = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
            
            // Use two points that make the path tangent to the previous path's end points
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
            
          }

 To make the work less complicated to the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates .

    // To make math easier, Shift car reference angle to 0 degrees. It means we are consider the car as the origin of our new coordinate 
          for (int i = 0; i < ptsx.size(); ++i){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
          }

In order to ensure more continuity on the trajectory (in addition to adding the last two point of the pass trajectory to the spline adjustment), the pass trajectory points are copied to the new trajectory. 

    // Start with all of the previous path points from last time
          for (int i = 0; i < previous_path_x.size(); ++i){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

The rest of the points are calculated by evaluating the spline and transforming the output coordinates to not local coordinates. Worth noticing the change in the velocity of the car from `double N = (target_dist / (0.02 * ref_vel / 2.24))`. The speed change is decided on the behavior part of the code, but it is used in that part to increase/decrease speed on every trajectory points instead of doing it for the complete trajectory.

    // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          
          double x_add_on = 0;
          
          // after filling it with previous points, we need to fill up the rest of our path planner. Here we will always output 50 points each cycle
          for (int i = 1; i < 50 - previous_path_x.size(); i++){
            double N = (target_dist / (0.02 * ref_vel / 2.24));  // From reference point to target point, the distance is evenly spaced into N parts; 2.24 is to convert mph to m/s
            double x_point = x_add_on + target_x / N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to normal because we rotated it eariler
            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

