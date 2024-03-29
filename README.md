# Ai_for_Robotics
- git clone --recurse-submodules _<url>_
## Modify file launch
 -  modify argument inside the file "/home/path_to_ws/src/environment_pkg/launch/simulate_tiago.launch"  **  
   arg name="path_to_ws" -->default="/path_to_your_ws"<--- subs this part with path to your ws
## launch the simulation
  - roslaunch environment_pkg simulate_tiago.launch
  - wait some minute for bring up

## How to use the simulation for projects
 - create a listener to "/joint_states" and listen to "wheel_left_joint" and "wheel_right_joint" position array to get wheel encoders
   message is sensor_msgs/JointState, so you need to import in the python file a "sensor_msgs/JointState" msg
 - To move the tiago create a publisher on "/mobile_base_controller/cmd_vel"
    - use in your python script a "geometry_msgs/Twist" msg and modify the x component of the msg "linear:  
       ---> x: 0.0  
       y: 0.0  
       z: 0.0  
     angular:  
       x: 0.0  
       y: 0.0  
       z: 0.0"
     
