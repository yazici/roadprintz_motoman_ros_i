# test_jog_cartesian

test node to move robot interactively to Cartesian goals;
 trajectory is still in joint space
 
 BETTER: test_move_cartesian:
 performs Cartesian moves w/rt the "truck" frame (which needs to be defined and published).
 
 

## Example usage
`rosrun test_jog_cartesian test_jog_cartesian`

Better:
Define a rear_wheel_frame with respect to the robot base_link.  Publish this transform with:
`roslaunch truck_model static_transforms.launch`
NOTE: need to get actual values from RoadPrintz vehicle!! (and reconcile w/ model)

Can start up Gazebo simulation with:
`roslaunch motoman_mh5020_description motoman_MH5020_on_truck_gazebo_sim.launch`

or start up the real robot (see roadprintz_motoman_ros_i package for instructions).

`rosrun test_jog_cartesian test_move_cartesian`
and respond to prompts (including simu vs physical)

## Running tests/demos
    
