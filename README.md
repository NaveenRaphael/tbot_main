# Turtlebots!
- [Turtlebots!](#turtlebots)
  - [Introduction](#introduction)
  - [Recommended references](#recommended-references)
  - [Instructions to use](#instructions-to-use)
    - [Setting up the computer](#setting-up-the-computer)
    - [Controlling an individual turtlebot](#controlling-an-individual-turtlebot)
    - [Controlling a swarm](#controlling-a-swarm)
  - [Files](#files)
    - [Service files:](#service-files)
    - [Message Files:](#message-files)
    - [Source files:](#source-files)
  - [Troubleshooting](#troubleshooting)
- [Appendix](#appendix)
  - [Username and password of the bot](#username-and-password-of-the-bot)
  - [Quick codes to copy paste:](#quick-codes-to-copy-paste)
  - [Changing router](#changing-router)
## Introduction
Hello there!
This will be a (almost) comprehensive guide to using the turtlebots in the DCL.
This is presently a work in progress

## Recommended references
It is recommended that you know a little bit about ros (see [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)) and a little bit about the specifications of the turtlebot (see [this link](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications)).

About ros, it is ncessary to know about rostopics, rosnodes and the basics of running a program in ros. RQT and RVIZ are tools that make interacting with ROS easier, and you can run these by typing the name of these tools in any terminal.

## Instructions to use
It is recommended to have `rqt` and `rviz` open while working with the robots. It is alsp recommended to follow the steps to control a swarm of turtlebots, since controlling a single turtlebot is but a specific case of the former. However, please do read the steps of both. 
### Setting up the computer
It is recommended to use a program that can open multiple instances of the terminal, like VS-code and X-terminal (use `ctrl+alt+t`).
1. In one instance of the terminal, run `roscore`
2. Have several instances of the terminal for the other programs to run
   1. In general, I have 8 terminals open, with 2 more terminals having run `rqt` and `rviz`
### Controlling an individual turtlebot
1. Switch on the robot, and connect to it via ssh (see [this](#Username-and-password-of-the-bot))
   1. There is a switch at the first "platform" of the turtlebot.
   2. `ssh` is a tool to open the terminal of the robot in the computer
2. Run the following in the terminal of the bot
   `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
   1. All the rostopics and rosnodes associated with the turtlebot will now be running. This can be seen in `rqt` 
3. In another terminal, run `roslaunch tbot_main basic_control.launch`. This allows for position and velocity control of the robot
4. In another terminal, run `rqt` (if you have not already). Open the service caller
   1. Velocity can be changed by providing a `v` and `w` in the service `change_tbot_velocity`
   2.  Position can be changed by providing `x` an `y` in the service `change_tbot_pos`

### Controlling a swarm
Note that `<i>` is used to indicate the ith bot.
Most of these commands can be copied from the [appendix](#quick-codes-to-copy-paste)
1. Switch on the robots and connect via ssh
2. Run the following command in each of the bots
 `ROS_NAMESPACE=<name_i> roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="<name_i>" set_lidar_frame_id:="<name_i>/base_scan"`
3. In step 3, run `ROS_NAMESPACE=<name_i> roslaunch tbot_main trajectory.launch x:=<x_pos_i> y:= <y_pos_i> tf:=<name_of_tf_frame _i>`
4. This requires a nodes to publish trajectory data. This is done in 2 ways;
   1. `rosrun tbot_main swarm_motion3`. Write and compile the code with the trajectory of the different robots 
   2. `ROS_NAMESPACE=<name_i> rosrun tbot_main P2T` converts a sequence of points to trajectories

## Files
### Service files:
1. [`positionInterrupt`](srv/positionInterrupt.srv) : (Not Implemented) To interrupt the `positionControl` rosnode of the turtlebot
2. [`positionRequest`](srv/positionRequest.srv) : "Requests" the turtlebot to move to the specified location
3. [`velocityInterrupt`](srv/velocityInterrupt.srv) : (Not Implemented) To interrupt the `velocityControl` rosnode of the turtlebot
4. [`velocityRequest`](srv/velocityRequest.srv) : "requests" the turtlebot to have the specified velocity

### Message Files:
1. [`trajectory`](msg/trajectory.msg): Handles trajectory motion needed for the trajectory control. Has the following:
   1. `x`: x-coordinate of the robot
   2. `y`: y-coordinate of the robot
   3. `t`: Orientation of the robot
   4. `v`: Velocity at that point on the trajectory
   5. `w`: Angular velocity at that point on the trajectory
### Source files:
1. [`softbody.hpp`](src/softbody.hpp) : (WIP) In line of a "Rigid Body"; the body has a position and orientation at every instant, but the distance between any point and the center can change. This was made to model a swarm; the position of every point on the swarm can be specified. 
2. [`FTP.py`](src/FTP.py) : (Fermat Toricelli Problem) Does 3 things:
   1. Identify the actual FTP solution and publishes in `tf`
   2. Identify the asymptotic solution of the FTP and publish in `tf`.
   3. Control the follower bot to follow the asymptotic solution
3. [`obstacle.hpp`](src/obstacle.hpp) : (WIP) (This was WIP a really long while back, and now I am not sure why I began this code)
4.  [`point.hpp`](src/point.hpp) : Contains the point class which contains useful functions to model 2D points for turtlebots
5.  [`positionController.cpp`](src/positionController.cpp) : Waits for the service [`positionRequest`](srv/positionRequest.srv) and calculated the velocity and omega to reach that point
    1.  The control law can be changed to suit the needs.
    2.  Also publishes the position of the bot in tf
    3.  Event-triggered control ready
6. [`swarm_motion.cpp`](src/swarm_motion.cpp) : (Deprecated) Implements the swarm as a "SoftBody". 
   1. The velocity and omega of the whole body can be controlled.
   2. The r and phi of each of the bot from the CoM can also be controlled
7. [`swarm_motion2.cpp`](src/swarm_motion2.cpp) : Implements the swarm as a "SoftBody". Uses trajectory control
8. [`swarm_motion3.cpp`](src/swarm_motion3.cpp) : A better swarm motion code that deprecates several previous codes.
9. [`velocityController.cpp`](src/velocityController.cpp) : Waits for the service [`velocityRequest`](srv/velocityRequest.srv)
   1. Event-triggered control ready
10. [`trajectory_tracking.cpp`](src/trajectory_tracking.cpp) : Implements trajectory tracking implemented by the paper referenced. Can be used to implement position control 
11. [`dynamic.cpp`](src/dynamic.cpp) : Implements Mrs. Rejitha's paper for obstacle avoidance. Presently only possible to run on tb0, because, yeah.
12. [`dynamic_base.cpp`](src/dynamic_base.cpp) : Base code for the `dynamic.cpp` Written to debug. Can be removed. 


## Troubleshooting
The following are the problems I have faced;
1. **Cannot find the IP of the bot** : Run `nmap -sn 192.168.10.100-199` to get the list of IP addresses. The list of IP addresses need to be changed depending on the present IP address of the bot
2. **IP Address changed** : If the IP address of the bot has changed, the following needs to be done after getting the new one and the connecting to it.
   1. open `~/.bashrc`
   2. make changes to the line which have ROS mentioned, then save and close
   3.  run `source ~/.bashrc`
3. Presently, the computers and the bots need to be connected to the router `CONTROL LAB 203C`. If it is not connected to this, please check the [appendix](##Changing-router)
4. The computer sometimes disconnects from the network. Not sure why or when. Reconnect Dongle to fix.
5. Turtlebot3 has faulty odometry readings.
6. Turtlebot1 has to be ... reinstalled with a monitor?


# Appendix
## Username and password of the bot
To run ssh, you need to first identify the ip-address of the robot:
   ```
   nmap -sn 192.168.10.100-199
   ```
The computer itself would be one of the ip addresses listed here, and should be 192.168.10.107. If not, check [troubleshooting](#troubleshooting), and edit this section with the new IP address.
On finding the ip address of the robot, run:
```
ssh ubuntu@<ip-address>
```
and then you connect to it with the password.
See [this](#quick-codes-to-copy-paste) for easy ways to copy paste the commands. 

These are presently the default. Say the IP address of the bot is `192.168.10.102`
```
username:   ubuntu@192.168.10.102
password:   turtlebot
``` 
## Quick codes to copy paste:
1. For identify the IP address of the robot:
   ```
   nmap -sn 192.168.10.100-199
   ```
   To connect to the robots:
   ```
   ssh ubuntu@192.168.10.100
   ssh ubuntu@192.168.10.101
   ssh ubuntu@192.168.10.102
   ssh ubuntu@192.168.10.103
   ssh ubuntu@192.168.10.104
   ssh ubuntu@192.168.10.105
   ssh ubuntu@192.168.10.106
   ssh ubuntu@192.168.10.108
   ssh ubuntu@192.168.10.109
   ssh ubuntu@192.168.10.110
   ```

2. For within the terminal of the bot
   ```
   ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_0" set_lidar_frame_id:="tb3_0/base_scan"
   
   ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_1" set_lidar_frame_id:="tb3_1/base_scan"
   
   ROS_NAMESPACE=tb3_2 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_2" set_lidar_frame_id:="tb3_2/base_scan"
   
   ROS_NAMESPACE=tb3_3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_3" set_lidar_frame_id:="tb3_3/base_scan"
   
   ROS_NAMESPACE=tb3_4 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_4" set_lidar_frame_id:="tb4_0/base_scan"
   
   ROS_NAMESPACE=tb3_5 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_5" set_lidar_frame_id:="tb3_5/base_scan"
   ```
3. For basic control
   ```
   ROS_NAMESPACE=tb3_0 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb0"

   ROS_NAMESPACE=tb3_1 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb1"

   ROS_NAMESPACE=tb3_2 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb2"

   ROS_NAMESPACE=tb3_3 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb3"

   ROS_NAMESPACE=tb3_4 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb4"

   ROS_NAMESPACE=tb3_5 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb5"
   ```
4. For Trajectory Control
   ```
   ROS_NAMESPACE=tb3_0 roslaunch tbot_main trajectory.launch x:=0 y:=0 tf:="tb0"

   ROS_NAMESPACE=tb3_1 roslaunch tbot_main trajectory.launch x:=0 y:=0 tf:="tb1"

   ROS_NAMESPACE=tb3_2 roslaunch tbot_main trajectory.launch x:=0 y:=0 tf:="tb2"

   ROS_NAMESPACE=tb3_3 roslaunch tbot_main trajectory.launch x:=0 y:=0 tf:="tb3"

   ROS_NAMESPACE=tb3_4 roslaunch tbot_main trajectory.launch x:=0 y:=0 tf:="tb4"

   ROS_NAMESPACE=tb3_5 roslaunch tbot_main trajectory.launch x:=0 y:=0 tf:="tb5"
   ```
## Changing router

Since the Turtlebots and Computer communicates over ROS via IP address, changing the router needs a lot of set-up. The following are needed.
1. Monitor
2. Keyboard
3. Name and password of the router

In the PC, the only changed are in the `~/.bashrc` file, with the corrected IP addresses. Do not forget to run `source ~/.bashrc` before running using the same terminal window.

For the turtlebot, a few steps are needed. 
1. Connect the monitor and keyboard to the turtlebot. (A mouse is not necessary)
2. Switch on the turtlebot. I usually switch this on while it is plugged in since I do not want the bot to switch off due to low battery when I am using, but I am not sure if this is necessary
3. Login:
   1. username: `ubuntu`
   2. password: `turtlebot`
4. Navigate to `/etc/netplan` and open the only file there
5. Edit the section with the name of the router and the password. Save and close
6. run `ip -a ` to check the new IP address.
7. (optional) You should change the IP addresses in the `bash` file as well. This can be done via ssh, so check [troubleshooting](#troubleshooting). 
