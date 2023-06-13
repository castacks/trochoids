![workflow status badge](https://github.com/castacks/trochoids/actions/workflows/ci-tests.yml/badge.svg)
# Time-Optimal Path Planning in a Constant Wind for Uncrewed Aerial Vehicles using Dubins Set Classification 

This repository contains code for the paper
**<a href="#">Time-Optimal Path Planning in a Constant Wind for Uncrewed Aerial Vehicles using Dubins Set Classification</a>**  
*<a href="https://sagars2.com">Sagar Sachdev\*</a>,
<a href="https://bradymoon.com">Brady Moon\*</a>,
<a href="https://theairlab.org/team/junbiny/">Junbin Yuan</a>,
<a href="https://www.ri.cmu.edu/ri-faculty/sebastian-scherer/">Sebastian Scherer</a><br/>
(\* equal contribution)*

<p align="center">   
    <img src="img/Fig1v7-2.png" alt="drawing" style="width:50%;"/>
</p>

## Brief Overview
 Time-optimal path planning in high winds for a
turning rate constrained UAV is a challenging problem to solve
and is important for deployment and field operations. Previous
works have used trochoidal path segments, which consist of
straight and maximum-rate turn segments, as optimal extremal
paths in uniform wind conditions. Current methods iterate
over all candidate trochoidal trajectory types and choose the
time-optimal one; however, this can be computationally slow.
As such, a method to narrow down the candidate trochoidal
trajectory types before computing the trajectories would reduce
the computation time. We thus introduce a geometric
approach to reduce the candidate trochoidal trajectory types by
framing the problem in the air-relative frame and bounding the
solution within a subset of candidate trajectories. This method
reduces overall computation by around 37% compared to pre-
existing methods in Bang-Straight-Bang trajectories, freeing
up computation for other onboard processes and can lead to
significant total computational reductions when solving many
trochoidal paths. When used within the framework of a global
path planner, faster state expansions help find solutions faster or
compute higher-quality paths.


## Prerequisites
* Ubuntu 18.04 or 20.04
* ROS Melodic or Noetic 
* Python 3.6+ (For visualizations)

### Building and Installation
* Clone this repo in your preferred directory
```
git clone git@bitbucket.org:castacks/trochoids.git
cd trochoids
```
* To build the unit tests and run them (optional), please run the following commands:
```
catkin build --make-args tests
source devel/setup.bash
```
Source the workspace (zsh version)
```
source devel/setup.zsh
```
And then launch the unit tests with
```
roslaunch trochoids unit_test.launch
```
This will run all the unit tests contained in unit_test_trochoid.cpp and unit_test_trochoid_classification.cpp. Examples of code usage can be found in the unit tests or details below.

## Usage

The main function for the trochoid solver is `get_trochoid_path()`. This function takes in the following parameters:

* Start State: [x, y, z, psi]
* Goal State: [x, y, z, psi]
* Wind: [x, y, z]
* Desired Speed (m/s)
* Max Kappa: 1/turning_radius (1/m)
* (Optional) Waypoint Distance: The distance between waypoints in the trochoid path (m)


If there is no wind, it solves for the path using a Dubins path, and if there is wind it uses a trochoidal path. 

### Simple Example
```cpp
double wind[3] = {0.3, 0.5, 0};
double desired_speed = 15;
double max_kappa = .02;
double waypoint_distance = 10;

trochoids::XYZPsiState start_state = {0, 0, 110, 0};
trochoids::XYZPsiState goal_state = {500, 0, 110, 0};

std::vector<trochoids::XYZPsiState> trochoid_path;
bool valid = trochoids::get_trochoid_path(start_state, goal_state, trochoid_path, wind, desired_speed, max_kappa, waypoint_distance);
```



## Citation
If you find this work useful, please cite our paper:
```

```