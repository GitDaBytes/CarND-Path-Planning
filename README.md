# CarND-Path-Planning-Project
This project is part of the Udacity Driving Car Engineer Nanodegree Program - Term 3

## Overview
The main goal of this project was to navigate a simulated vehicle along a freeway in a safe manner, while maintaining comfort for vehicle occupants, yet maintaing good efficient forward progress.

## Approach
While the general approach seemed simple, generate a series of waypoints in front of the car for the car to follow. To give an example as to how this works, if we laid a series of waypoints down the middle of one of the lanes on the freeway at constant spacing then the car would drive at a constant velocity. If we increase, or decrease the distance between each point as we move down the road, the car will accelerate or decelerate. Similarly, if we move the points from one lane to another, the car will turn into the new lane.

### Comfort
As you can imagine, if we space out the waypoints too far, in either the x, or y directions, we end up making the car accelerate or decelerate too quickly, or shaking the car to violently from one side to the other causing a bad experience for the passengers. We refer to this excessive acceleration(s) in one direction or the other as 'jerk'.

In order to minimize jerk... I started by applying the technique that was discussed in the project walkthrough, which seemed like a good idea. The basic idea is to build a path from current vehicle position until about one second into the future. To keep updates smooth is a little tricky. The idea is to not throw away previous estimates on each update loop, but rather to add new points onto the end of the current selected path. We can further smooth the path by fitting a spline to the new points, then resampling from that spline to give new postions. We use a fixed acceleration rate if we want to speed up or slow down... for the general case (more on that later). Using fixed acceleration and spline fitting minimiezd jerk in latitudinal and longitudunal directions.

Note: I should mentioned that in order to work with waypoints and planning, it is easiest to convert all car location points to s and d. Think of these as road based coordinates vs. map based. s is the distance up the road and d is the distance from the left hand side of the road. Planning in these coordinates means you dont have to worry so much about dealing with bends in roads and makes the computation much, much simpler.

### Forward Progress
Now we can drive smoothly around the track, which is great, we have the issue that there are quite a number of cars driving around us in all lanes and at different speeds. They change lanes and generally get in the way. Our goal is to change lanes and get around them to keep moving. Luckily we are provided real-time updates of vehicle data for teh vehicles sensed around us from the perception systems. We can tell the cars' speed, position, and id.

The first thing I dealt with is braking so i didnt run into the back of a car in my lane. First of all, each loop, I check which car is in my lane. I am adding new data points onto my running list, and hence if my new data point is 1 second in the future, I calculate where the detected car in my lane will be one second in the future. If it is within a pre-defined distance from me, I slow our car down by the greater of a pre-determined deceleration, or the rate the car detected is slowing. In order to deal with the latter I keep a running history of all detected car's data so I can compare current car data against the previous data to compute acceleration etc.

Once I was able to avoid running into the back of all cars, I worried about changing lanes. To tackle this, I iteratively made my approach more and more refined, which came with added complexity, but seems to work well. I ended up keeping a history of all cars' states for detected cars, so I could calulate their acceleration on the next loop. I then would track which lane all cars were in. Now, once a car got within a threshold safety distance in front of me, I triggered my switch lane routine as follows:

* Check the lane to the left and right of current vehicle and iterate for each car (if we are on the left or right lane, we of course only check one valid lane),
* Compute where each car will be in that lane one second from now, taking into account car postion, velocity and current acceleration (assumes fixed acceleration).
* If the lane is clear ahead of our target point (one second from now) move into that lane (left / outside lane always prefered per US regulations), move into that lane
* If a car is in that lane (we only check for cars around our predicted position - behind and in front - plus a safety margin, plus a small buffer) move into that lane as long as a car ahead of our target pos in that lane is not breaking, and is going faster than we currently are going (otherwise what is the point),
* Finally, if a new lane could not be identified safely, start braking.


### Room for Improvement?
The project is challenging, yet quite a fun one to implement. Ultimately my car would navigate the track seemingly quite safely while negotiating traffic, favoring overtaking on the left where possible and seemingly when safe to do so. I am sure there are instances where my car could get caught out such as if a car came up to us at high speed in the lane I am moving to, or if a car swerved in front and hit the brakes. I think this could be dealt with by overriding the comfort algorithms and clearing out the waypoint history to slam on the brakes etc. 

---
### Udacity Project Instructions
The original Udacity project instructions are incoporated from this point forward:
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

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

