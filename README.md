# Ai_for_Robotics
- git clone --recurse-submodules <url>
## Modify file launch /home/path_to_ws/src/environment_pkg/launch/simulate_tiago.launch 
 -  modify argument
   <arg name="path_to_ws" default="/path_to_your_ws"/> -> with path to your ws
## launch the simulation
  - roslaunch environment_pkg simulate_tiago.launch
  - wait some minute for bring up
