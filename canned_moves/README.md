# canned_moves
Some hard-coded moves, useful e.g. to fold/unfold the robot for travel

## Example usage
To move the robot from travel pose to home pose, run:
`rosrun canned_moves travel_pose_to_home_pose`
This has a max tool height of 3.047m (10 feet).
NEED 10' CLEARANCE TO UNFOLD.
AlSO, robot will fully extend, so need x-direction clearance behind bumper.

Note: if robot is not initially close to travel pose (for joints 1,2,3), node will quit with no motion.

To move robot back to travel pose (re-fold), start from home pose (all 0 angs) and run:
`rosrun canned_moves home_pose_to_travel_pose`
Also requires a 10' clearance.
Note: if robot is not initially close to home pose (for joints 1,2,3), node will quit with no motion.

Travel pose has elbow at approx 8'2" above ground.


    
