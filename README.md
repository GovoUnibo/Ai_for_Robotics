# Ai_for_Robotics
## Intallation
 -  git clone https://github.com/GovoUnibo/Ai_for_Robotics
### if submodule is required
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
    - use in your python script a "geometry_msgs/Twist" msg and modify the linear x component of the msg for a linear movement or angular z component for a angular movement
    - "linear:  
       ---> x: 0.0  
       y: 0.0  
       z: 0.0  
     angular:  
       x: 0.0  
       y: 0.0  
       ---> z: 0.0"

## How to use RosServices for Final Challenge

### Box Services:
- BOX1/pick_up: Picks up a box. (__If the box is not inside a circle of raw 1.5m, the box won't be picked up__)
- BOX1/put_down: Puts down a box.
- BOX1/update_position: Updates the position of a box.
- BOX1/get_position: Retrieves the position of a box.  
(Similar services exist for BOX2 and BOX3.)
### Message Box
 - BoxPickUp BoxPutDown
   - request and responce are bools
 - BoxUpdatePos
   - request: int32, float32 bool
   - responce bool
 - BoxPos
   - request bool
   - responce float32, float32, float32 indicating x,y,z pos variable 
### Door Services:
- Door1/update_pos: Opens or closes Door1.
- Door2/update_pos: Opens or closes Door2.
- Door3/update_pos: Opens or closes Door3.
- Door4/update_pos: Opens or closes Door4.
### Message Box
 - DoorOpen
    - request and responce are bools