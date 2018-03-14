# Udacity Robotics Software Engineer Project
## Term2 SLAM project Lab

## Abstract
This project created the 2D occupancy grid and 3D octomap from a provided kitchen-dinning and auther created rooms-objects simulated environment. It used RTAB-Map package, the best solution for simultaneous localization and mapping (SLAM) algorithm, to develop robots that can map environments in ROS. The results for both   simulated environments will be disciussed in this article.

<img src="images/new_robot_result-w1.PNG" width="60%" height="60%" title="Test result">

## Introduction
For robotic mapping and navigation fields, simultaneous localization and mapping (SLAM) is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it. While this initially appears to be a chicken-and-egg problem there are several algorithms known for solving it, at least approximately, in tractable time for certain environments. Popular approximate solution methods include the particle filter, extended Kalman filter, and GraphSLAM.

SLAM algorithms are tailored to the available resources, hence not aimed at perfection, but at operational compliance. Published approaches are employed in self-driving cars, unmanned aerial vehicles, autonomous underwater vehicles, planetary rovers, newly emerging domestic robots and even inside the human body.[1]

The project focuses on the following several tasks:

* Creating a Catkin workspace under Robot Operatiosn System (ROS) with all launch, worlds, urdf and scripts folders. Installing rtabmap package.
* Building a mobile robot with RGBD camera and laser scanner for simulated tasks.
* Creating a ROS package that launches a robot model in a Gazebo world and utilizes packages like rtabmap, teleop and the RViz.
* Exploring, adding, and tuning specific parameters corresponding to each package to achieve the best possible mapping results.

## Background
Robotic mapping: the goal for an autonomous robot is to be able to construct (or use) a map or floor plan and to localize itself in it[2].

SLAM addresses the main perception problem of a robot navigating an unknown environment. While navigating the environment, the robot seeks to acquire a map thereof, and at the same time it wishes to localize itself using its map. The use of SLAM problems can be motivated in two different ways: one might be interested in detailed environment models, or one might seek to maintain an accurate sense of a mobile robot’s location. SLAM serves both of these purposes.

SLAM are derived: (1) the extended Kalman filter (EKF); (2) particle filtering; (3) graph optimization. 

SLAM algorithms generally fall into these five categories: 
1. Extended Kalman filter SLAM (EKF) 
2. Sparse Extended Information Filter (SEIF)
3. Extended Information Form (EIF)
4. FastSLAM 
5. GraphSLAM 

FastSLAM 
The main advantage of the FastSLAM algorithm is that it uses a particle filter approach to solve the SLAM problem. Each particle will hold a guess of the robot trajectory, and by doing so, the SLAM problem is reduced to mapping with known poses. But, in fact, this algorithm presents a big disadvantage since it must always assume that there are known landmark positions, and thus with FastSLAM the robot is not able to model an arbitrary environment. 

Grid-based FastSLAM
The grid mapping algorithm can model the environment using grid maps without predefining any landmark position. So by extending the FastSLAM algorithm to occupancy grid maps, it can now solve the SLAM problem in an arbitrary environment. While mapping a real-world environment, the mobile robots mostly equipped  with range sensors. The FastSLAM algorithm can be extended and solved the SLAM problem in term of grid maps.

Sampling Motion-p(x_{t} | x_{t-1}^{[k]} , u_{t})p(x 
t
​	 ∣x 
t−1
[k]
​	 ,u 
t
​	 ): Estimates the current pose given the k-th particle previous pose and the current controls u.
Map Estimation-p(m_{t} | z_{t}, x_{t}^{[k]} , m_{t-1}^{[k]})p(m 
t
​	 ∣z 
t
​	 ,x 
t
[k]
​	 ,m 
t−1
[k]
​	 ): Estimates the current map given the current measurements, the current k-th particle pose, and the previous k-th particle map
Importance Weight-p(z_{t} | x_{t}^{[k]} , m^{[k]})p(z 
t
​	 ∣x 
t
[k]
​	 ,m 
[k]
 ): Estimates the current likelihood of the measurement given the current k-th particle pose and the current k-th particle map.

The robot mapping performance is related a running environment directly, it is so important which hardware and virtual machine configuration were used in this project.

### Hardware:
     Computer model: Surface Pro 4

     Processor: Intel i7-6650U CPU @ 2.20GHz @2.21GHz

     RAM: 16GB

     Operation System: Window 10 Pro

### Virual Machine:

     VMware Workstation 12 Pro, version 12.5.6

     Processor: 2

     Memory: 13.67GB

     Hard Disk: 40 GB

### Software

1. Using an Udacity ROS (Kinetic) package to create a robot simulation environment on VMWare machine. 
This ROS includes Python (2.7), Gazebo (7.10.0) and RViz (1.12.15) packages.

2. Using URDF (Unified Robot Description Format) to create the robot model which includes pose, inertial, collision and visual data.  
Two sensors - a camera and a laser rangefinder (Hokuyo)[1] was added in this URDF model.

3. The Kitchen-Dinning map model is used in first part of the project, and the second map model is created by auther which called rooms-objects model.

<img src="images/map.PNG" width="30%" height="30%" title="Maze Map">

4. RTAB-Map (Real-Time Appearance-Based Mapping) approach based on a global loop closure detector with real-time constraints. It is used to generate a 3D point clouds of the environment and/or to create a 2D occupancy grid map for navigation.

5. A Python code - teleop.py  was used to send a moving direction messages to the robot.


3. Compare Monte Carlo Simulations vs. Extend Kalman Filters

|    | MCL | EKF |
| :--- | :---: | :---: |
| Measurements | Raw Measurements | Landmarks |
| Measurement Noise | Any | Guassian |
| Posterior | Particles | Guassian |
| Efficiency(memory) | OK | Good |
| Efficency(time) | OK | Good |
| Ease of implementation | Good | OK |
| Resolution | OK | Good |
| Robustness | Good | Poor |
| Memory & Resolution Control | Yes | No |
| Global Localization | Yes | No |
| State Space | Multimodel Discrete | Unimodel Continuous |




## Results
### Testing scenario:
Both robots used the same map with same starting (0 0 -0.785) and target (0.995 -2.99 0) position.

| udacity_bot | new_robot |
| :---: | :---: |
| <img src="images/udacity_robot_w00.PNG" width="60%" height="30%" title="Starting udacity_bot"> | <img src="images/new_robot_w01.PNG" width="50%" height="25%" title="Starting new_robot"> |

### Testing results
#### Both robots navigated in map well and could arrive to the target position within reasonable time.
| | udacity_bot | new_robot |
| :---: | :---: | :---: |
| Go straight | <img src="images/udacity_robot_w01.PNG" width="60%" height="24%" title="Go udacity_bot"> | <img src="images/new_robot_w02.PNG" width="50%" height="16%" title="Go new_robot"> |
| Make a turn | <img src="images/udacity_robot_w02.PNG" width="60%" height="24%" title="Make a turn udacity_bot"> | <img src="images/new_robot_w04.PNG" width="50%" height="16%" title="Make a turn new_robot"> |
| Arrived target | <img src="images/udacity_robot_w04.PNG" width="60%" height="24%" title="Arrived target udacity_bot"> | <img src="images/new_robot_w_result.PNG" width="50%" height="16%" title="Arrived target new robot"> |
| Average Time | 6 -7 munites | 4 -5 munites |

The navigation trajectory for both robots is a green line route, the robots arrived to the goal in the end. 
The problem is that robots need to go up then make a cycle turn first in the map (Figure 1) and this cycle turn routing wasted time.
The better navigation approach is followed red line, it goes to the target position directly.  
Anther problem is the robot stuck on the wall in several testings. The program needs to restart to solve this issue.



####  Figure 1.   <img src="images/new_map.PNG" width="50%" height="50%" title="Maze Map">




## Model Configuration

### These parameters were adjusted in the project to improve the robot performance:

 * /amcl/laser_model_type: likelihood_field_prob
 = (string, default: "likelihood_field") Which model to use, either beam, likelihood_field, or likelihood_field_prob (same as likelihood_field but incorporates the beamskip feature, if enabled)[5].

    Used likelihood_field_prob, make laser sensor has beamskip feature.

 * /amcl/max_particles: 240
 =  (int, default: 5000) Maximum allowed number of particles.
 * /amcl/min_particles: 30
 = (int, default: 100) Minimum allowed number of particles[5].

    Adjusted these two value lower to reduce CPU usage and improve performance.

 * /amcl/resample_interval: 1.0
 = (int, default: 2) Number of filter updates required before resampling[5].

    Set a lower value to improve performance.

 * /amcl/transform_tolerance: 3.2
 =  (int, default: 2) Number of filter updates required before resampling[5].

    Set the value higher to improve localization accuracy.

 * /move_base/TrajectoryPlannerROS/sim_time: 3.0
 = (double, default: 1.0) The amount of time to forward-simulate trajectories in seconds[5].

    Set higher value to speed up robot navigation.

 * /move_base/TrajectoryPlannerROS/xy_goal_tolerance: 0.05
 = (double, default: 0.10) The tolerance in meters for the controller in the x & y distance when achieving a goal[5].

    Reduce the value to increase the challenge to achiev a goal

 * /move_base/controller_frequency: 5.0
 = (double, default: 20.0) The frequency at which this controller will be called in Hz. Uses searchParam to read the parameter from parent namespaces if not set in the namespace of the controller. For use with move_base, this means that you only need to set its "controller_frequency" parameter and can safely leave this one unset[5]. 

    Set the lower value to eliminate the warning message "Control loop missed its desired rate of 20.0000Hz". This parameter doesn't impact robot performance, but it will reduce these unnecessary warning messages on the screen and in the log file.

 * /move_base/global_costmap/raytrace_range: 9.0
 * /move_base/local_costmap/raytrace_range: 9.0
 = (double, default: 3.0) The maximum range in meters at which to raytrace out obstacles from the map using sensor data[5].

    Used higher value to increase sensor detecting obstacles distance

 * /move_base/global_costmap/robot_radius: 0.19
 * /move_base/local_costmap/robot_radius: 0.19
 = (double, default: 0.46)  The radius of the robot in meters, this parameter should only be set for circular robots, all others should use the "footprint" parameter[5].

    Set a lower value to match the project robot size.

 * /move_base/global_costmap/transform_tolerance: 0.4
 * /move_base/local_costmap/transform_tolerance: 0.4
 =  (double, default: 0.2) Specifies the delay in transform (tf) data that is tolerable in seconds. This parameter serves as a safeguard to losing a link in the tf tree while still allowing an amount of latency the user is comfortable with to exist in the system[5].

    Set a little higher value to increase the system tolerable.




## Discussion

* Adjusting the parameter is a big challenge and time consuming job. Those parameters can be changed independently, but they are related eachother. It is impossible that one person tries all possible combination values for all parameters in limited time. A team work needs to assign for achieving the best result.

* AMCL would'n work well for the kidnapped robot problem. The data from laser sensor can help to detect the current new location again, but it is not guarantee that robot gets correct location or takes too long to find a new position.

* A moving robot with MCL/AMCl algorithm can be used warehouse industry to move and delivery good inside the warehouse. This job and working environment have clear start and end positions. 




## Future Work

* Both robots started forward to dead end direction, then turned back to reverse point (Figure 1). The further study needs to involve to find out this is an algorithm issue or parameter turning problem.

* Additional sensor can be added on back of the robot, so the robot can go back and forth without rotating to navigate to the target position.

* Adjusting and trying different parameters are very man power cost work, a database can be built to store these test parameter and result timing data to help adjusting the parameters in the new robots, and use Deep Learning technology to figure out and generate these parameters automatically.



## Reference

[1] Wikipedia, "Simultaneous localization and mapping" https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping, 2018.

[2] Wikipedia, “Robotic mapping” https://en.wikipedia.org/wiki/Robotic_mapping, 2017.

[3] Wikipedia, "Kalman filter" https://en.wikipedia.org/wiki/Kalman_filter 2018

[4] Wikipedia, "Monte Carlo localization" https://en.wikipedia.org/wiki/Monte_Carlo_localization 2018

[5] wiki.ROS.ORG, "Documentation" http://wiki.ros.org/  2018

[6] R. Siegwart, "Mobile Robot Localization" http://www.cs.cmu.edu/~rasc/Download/AMRobots5.pdf 2002

[7] Zuozhi Yang and Todd W. Neller, "A Monte Carlo Localization Assignment
Using a Neato Vacuum with ROS" https://aaai.org/ocs/index.php/AAAI/AAAI17/paper/download/15025/13983 2017

[8] wikibooks.org , "Robotics/Navigation/Localization" https://en.m.wikibooks.org/wiki/Robotics/Navigation/Localization 
