# Canfield
Files relating to the ROS implementation of the Canfield joint.

# Launch Gazebo
roslaunch gazebo_ros empty_world.launch

# Set Gravity to Zero
rosservice call / gazebo / set_physics_properties 0.001 1000.0 ’
[0.0 , 0.0 , 0.0] ’ ’[False , 0, 50, 1.3 , 0.0 , 0.001 , 100.0 ,
0.0 , 0.2 , 20] ’

# Set Gravity to Normal
rosservice call / gazebo / set_physics_properties 0.001 1000.0 ’
[0.0 , 0.0 , 0.0] ’ ’[False , 0, 50, 1.3 , 0.0 , 0.001 , 100.0 ,
0.0 , 0.2 , 20] ’

# Spawn Model
cd ~/ catkin_ws / src/ Canfield
rosrun gazebo_ros spawn_model -file canfield .sdf -sdf -x 0 -y
0 -z 1 -model Canfield

# Perturb Model from Command Line
rosservice call / gazebo / apply_joint_effort joint1 -- 10000 0
1000000
