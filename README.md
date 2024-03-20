# Ai_for_Robotics
- git clone --recurse-submodules _<url>_
## Modify file launch
 -  modify argument inside the file "/home/path_to_ws/src/environment_pkg/launch/simulate_tiago.launch"  **
   arg name="path_to_ws" -->default="/path_to_your_ws"<--- subs this part with path to your ws
## launch the simulation
  - roslaunch environment_pkg simulate_tiago.launch
  - wait some minute for bring up
