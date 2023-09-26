# !/bin/sh

# Replace ".bash" with your shell if you're not using bash:
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Sourcing ROS 2 setup files will set several environment variables necessary for operating ROS 2:
printenv | grep -i ROS

# Once you have determined a unique integer for your group of ROS 2 nodes, you can set the environment variable with the following command:
export ROS_DOMAIN_ID=10
echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
export ROS_LOCALHOST_ONLY=1
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

# -------- Run the below code in a new terminal tab --------

# Install the turtlesim package for your ROS 2 distro:
sudo apt update
sudo apt install ros-humble-turtlesim

# Check that the package is installed:
ros2 pkg executables turtlesim

# To start turtlesim:
ros2 run turtlesim turtlesim_node

# -------- Run the below code in a new terminal tab --------

# Now run a new node to control the turtle in the first node:
ros2 run turtlesim turtle_teleop_key

# -------- Run the below code in a new terminal tab --------

# Install rqt and its plugins:
sudo apt update
sudo apt install ~nros-humble-rqt*

# To run rqt:
rqt

ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

# -------- Run the below code in a new terminal tab --------

# The command ros2 run launches an executable from a package:
ros2 run turtlesim turtlesim_node

# Reassign the name of our /turtlesim node:

ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# Now that you know the names of your nodes, you can access more information about them with:
ros2 node info /my_turtle

# -------- Run the below code in a new terminal tab --------

# In a new terminal:
ros2 run turtlesim turtlesim_node

# -------- Run the below code in a new terminal tab --------

# Open another terminal and run:
ros2 run turtlesim turtle_teleop_key

# -------- Run the below code in a new terminal tab --------

# Run rqt_graph:
rqt_graph

# To see the data being published on a topic, use:
ros2 topic echo /turtle1/cmd_vel

# Another way to look at this is running:
ros2 topic info /turtle1/cmd_vel

# The topic types we saw earlier after running ros2 topic list -t let us know what message type is used on each topic. Recall that the cmd_vel topic has the type:
geometry_msgs/msg/Twist

# Now we can run ros2 interface show <msg type> on this type to learn its details. Specifically, what structure of data the message expects.
ros2 interface show geometry_msgs/msg/Twist

# Now that you have the message structure, you can publish data onto a topic directly from the command line using:
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# Finally, you can run echo on the pose topic and recheck rqt_graph:
ros2 topic echo /turtle1/pose

# For one last introspection on this process, you can view the rate at which data is published using:
ros2 topic hz /turtle1/pose

# -------- Run the below code in a new terminal tab --------

# In a new terminal:
ros2 run turtlesim turtlesim_node

# Open another terminal and run:
ros2 run turtlesim turtle_teleop_key

# Let’s take a look at turtlesim’s /clear service:
ros2 service type /clear

# To see the types of all the active services at the same time, you can append the --show-types option, abbreviated as -t, to the list command:
ros2 service list -t

# If you want to find all the services of a specific type:
ros2 service find std_srvs/srv/Empty

# Try this on the /clear service’s type, Empty:
ros2 interface show std_srvs/srv/Empty

# To see the request and response arguments of the /spawn service:
ros2 interface show turtlesim/srv/Spawn

# Now that you know what a service type is, how to find a service’s type, and how to find the structure of that type’s arguments, you can call a service using:
ros2 service call /clear std_srvs/srv/Empty

# Now let’s spawn a new turtle by calling /spawn and setting arguments. Input <arguments> in a service call from the command-line need to be in YAML syntax:
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"

# -------- Run the below code in a new terminal tab --------

# In a new terminal:
ros2 run turtlesim turtlesim_node

# Open another terminal:
ros2 run turtlesim turtle_teleop_key

# To see the parameters belonging to your nodes, open a new terminal and enter the command:
ros2 param list

# To display the type and current value of a parameter, use the command:
ros2 param get /turtlesim background_g

# To change a parameter’s value at runtime, use the command:
ros2 param set /turtlesim background_r 150

# You can view all of a node’s current parameter values by using the command:
ros2 param dump /turtlesim > turtlesim.yaml

# Stop your running turtlesim node, and try reloading it with your saved parameters, using:
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml

# -------- Run the below code in a new terminal tab --------

# In a new terminal:
ros2 run turtlesim turtlesim_node

# Open another terminal and run:
ros2 run turtlesim turtle_teleop_key

# To identify all the actions in the ROS graph, run the command:
ros2 action list

# Actions have types, similar to topics and services. To find /turtle1/rotate_absolute’s type, run the command:
ros2 action list -t

# You can further introspect the /turtle1/rotate_absolute action with the command:
ros2 action info /turtle1/rotate_absolute

# Now let’s send an action goal from the command line with the following syntax:
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"

# -------- Run the below code in a new terminal tab --------

# Start rqt_console in a new terminal with the following command:
ros2 run rqt_console rqt_console

# Now start turtlesim in a new terminal with the following command:
ros2 run turtlesim turtlesim_node

# To produce log messages for rqt_console to display, let’s have the turtle run into the wall. In a new terminal, enter the ros2 topic pub command (discussed in detail in the topics tutorial) below:
ros2 topic pub -r 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}"

# You can set the default logger level when you first run the /turtlesim node using remapping. Enter the following command in your terminal:
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN

# -------- Run the below code in a new terminal tab --------

# In a new terminal:
ros2 launch turtlesim multisim.launch.py

# Now that these nodes are running, you can control them like any other ROS 2 nodes. For example, you can make the turtles drive in opposite directions by opening up two additional terminals and running the following commands: In the second terminal:
ros2 topic pub  /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# In the third terminal:
ros2 topic pub  /turtlesim2/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"

# -------- Run the below code in a new terminal tab --------

# In a new terminal:
ros2 run turtlesim turtlesim_node

# Open another terminal and run:
ros2 run turtlesim turtle_teleop_key

# Let’s also make a new directory to store our saved recordings, just as good practice:
mkdir bag_files
cd bag_files

# ros2 bag can only record data from published messages in topics. To see the list of your system’s topics, # In a new terminal the command:
ros2 topic list

# To see the data that /turtle1/cmd_vel is publishing, run the command:
ros2 topic echo /turtle1/cmd_vel

# To record the data published to a topic use the command syntax:
ros2 bag record /turtle1/cmd_vel

# You can also record multiple topics, as well as change the name of the file ros2 bag saves to:
ros2 bag record -o subset /turtle1/cmd_vel /turtle1/pose

# You can see details about your recording by running:
ros2 bag info subset

# Before replaying the bag file, enter Ctrl+C in the terminal where the teleop is running. Then make sure your turtlesim window is visible so you can see the bag file in action:
ros2 bag play subset

# To get an idea of how often position data is published:
ros2 topic hz /turtle1/pose
