# ROS Concepts - Robotic Arm Basics (Note 2)

## Message Interface

The standard interface to a robotic arm in ROS is

`control_msgs/FollowJointTrajectory ( follow_joint_trajectory *action*)`
_Command a trajectory for the arm and monitor its progress._

`sensor_msgs/JointState ( joint_states *topic*)`
_Publish the current state of each joint in the arm._

*Note: Ensure you know what a __topic__ and __action__ mean in ROS [From ROS Basics 1]*

The `follow_joint_trajectory`/ `joint_states` ROS interface allows us to, in a
portable manner, observe and command a robot arm’s joints.

## Hardware/Driver

To implement the follow_joint_trajectory / joint_states interface for a physical robot, we need to write a node that will communicate with the robot hardware.

Ideally, we can find a reusable library that implements the protocol, which we can wrap into a ROS node where we will handle any necessary data transformations, such as unit conversions.

We need to write a driver(a ROS Node) which supports the above mentioned action/topic interface(`follow_joint_trajectory` / `joint_states`)

## Modeling(URDF)

We need to either generate a URDF from CAD or write one ourselves. Detailed tutorials for URDF generation are available.

1. URDF from CAD models
    - http://wiki.ros.org/sw_urdf_exporter/Tutorials
    - http://wiki.ros.org/sw_urdf_exporter/Tutorials/Export%20an%20Assembly
2. URDF from scratch - http://wiki.ros.org/urdf/Tutorials

## Using the model in Gazebo

We can import the URDF into gazebo using a launch file like this
```
<launch>
    
    <param name="robot_description" textfile="$(find cougarbot)/cougarbot.urdf" />
    
    <include file="$(find gazebo_ros)/launch/empty_world.launch"/>
    
    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
    args="-param robot_description -urdf -model cougarbot" />
</launch>
```

We need something to help us control all its joints via the `follow_joint_trajectory / joint_states` interface. For this purpose, we’re going to use two plugins: the `ros_control` plugin will accept new desired trajectories via `follow_joint_trajectory` , while the `ros_joint_state_publisher` will publish the `joint_states` data.


