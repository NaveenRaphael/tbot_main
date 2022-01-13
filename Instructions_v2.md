# Turtlebots!
## Introduction
Hello there!
This will be a comprehensive guide to using the turtlebots in the DCL.
This is presently a work in progress

## Instructions to use
The following steps are for the most basic control; to control a swarm, please read the next section
1. Switch on the robot, and connect to it via ssh (see [this]())
2. Run the following in the terminal of the bot
   `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
3. In another terminal, run `roslaunch tbot_main basic_control.launch`. This allows for position and velocity control of the robot
4. In another terminal, run `rqt`. Open the service caller
   1. Velocity can be changed by providing a `v` and `w` in the service `change_tbot_velocity`
   2.  Position can be changed by providing `x` an `y` in the service `change_tbot_pos`

To control a swarm; Note that `<i>` is used to indicate the ith bot.
1. In step 2: run this instead `ROS_NAMESPACE=<name_i> roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="<name_i>" set_lidar_frame_id:="<name_i>/base_scan"`
2. In step 3, run `ROS_NAMESPACE=<name_i> roslaunch tbot_main basic_control.launch x:=<x_pos_i> y:= <y_pos_i> tf:=<name_of_tf_frame _i>`
3. The names of the services will be different based on the names given.
4. Additionally, the `.launch` file `launch/circle_num.launch` is a work in progress to control the swarm in specific manners coded in `src/swarm_motion.cpp`. If these files are editted, run `catkin_make`   

## Files
### Service files:
1. [`positionInterrupt`](srv/positionInterrupt.srv) : (Not Implemented) To interrupt the `positionControl` rosnode of the turtlebot
2. [`positionRequest`](srv/positionRequest.srv) : "Requests" the turtlebot to move to the specified location
3. [`velocityInterrupt`](srv/velocityInterrupt.srv) : (Not Implemented) To interrupt the `velocityControl` rosnode of the turtlebot
4. [`velocityRequest`](srv/velocityRequest.srv) : "requests" the turtlebot to have the specified velocity

### Source files:
1. [`follow_path.cpp`](src/follow_path.cpp) : (Deprecated- see [`swarm_motion.cpp`](###Source-files)) Give the number of turtlebots, center of circle to orbit and radius of circle, and watch the turtlebots follow this
2. [`FTP.py`](src/FTP.py) : (WIP) Does 3 things:
   1. Identify the actual FTP solution and publishes in `tf`
   2. Identify the asymptotic solution of the FTP and publish in `tf`.
   3. Control the follower bot to follow the asymptotic solution (WIP)
3. [`plainVelocityControl.cpp`](src/plainVelocityControl.cpp) : (Dropped - will be deleted)
4.  [`point.hpp`](src/point.hpp) : Contains the point class which contains useful functions to model 2D points for turtlebots
5.  [`positionController.cpp`](src/positionController.cpp) : Waits for the service [`positionRequest`](srv/positionRequest.srv) and calculated the velocity and omega to reach that point
    1.  The control law can be changed to suit the needs.
    2.  Also publishes the position of the bot in tf
    3.  Event-triggered control ready

6. [`swarm_motion.cpp`](src/swarm_motion.cpp) : Implements the swarm as a "SoftBody". 
   1. The velocity and omega of the whole body can be controlled.
   2. The r and phi of each of the bot from the CoM can also be controlled
7. [`velocityController.cpp`](src/velocityController.cpp) : Waits for the service [`velocityRequest`](srv/velocityRequest.srv)
   1. Event-triggered control ready 


## Troubleshooting
The following are the problems I have faced;
1. **Cannot find the IP of the bot** : Run `nmap -sn 192.168.10.100-199` to get the list of IP addresses. The list of IP addresses need to be changed depending on the present IP address of the bot
2. **IP Address changed** : If the IP address of the bot has changed, the following needs to be done after getting the new one and the connecting to it.
   1. open `~/.bashrc`
   2. make changes to the line which have ROS mentioned, then save and close
   3.  run `source ~/.bashrc`
3. Presently, the computers and the bots need to be connected to the router `CONTROL LAB 203C`. If it is not connected to this, please check the [appendix](##Changing-router)


Todo
---
1. Verify the `src/FTP.py` works
2. Make a map using the LIDAR data from which the obstacle data can be extracted easily. Needed for the "old" obstacle avoidance code

 
# Appendix
## Username and password of the bot
These are presently the default. Say the IP address of the bot is `192.168.10.102`
```
username:   ubuntu@192.168.10.102
password:   turtlebot
``` 
## Quick codes to copy paste:
1. For within the terminal of the bot
   ```
   ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_0" set_lidar_frame_id:="tb3_0/base_scan"
   
   ROS_NAMESPACE=tb3_1 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_1" set_lidar_frame_id:="tb3_1/base_scan"
   
   ROS_NAMESPACE=tb3_2 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_2" set_lidar_frame_id:="tb3_2/base_scan"
   
   ROS_NAMESPACE=tb3_3 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_3" set_lidar_frame_id:="tb3_3/base_scan"
   
   ROS_NAMESPACE=tb3_4 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_4" set_lidar_frame_id:="tb4_0/base_scan"
   
   ROS_NAMESPACE=tb3_5 roslaunch turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:="tb3_5" set_lidar_frame_id:="tb3_5/base_scan"
   ```
2. For basic control
   ```
   ROS_NAMESPACE=tb3_0 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb0"

   ROS_NAMESPACE=tb3_1 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb1"

   ROS_NAMESPACE=tb3_2 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb2"

   ROS_NAMESPACE=tb3_3 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb3"

   ROS_NAMESPACE=tb3_4 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb4"

   ROS_NAMESPACE=tb3_5 roslaunch tbot_main basic_control.launch x:=0 y:=0 tf:="tb5"
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