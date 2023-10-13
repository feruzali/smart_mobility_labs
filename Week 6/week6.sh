# !/bin/sh

# Navigate into the ros2_ws/src/cpp_pubsub/src folder, created in the previous tutorial, and download the example talker code by entering the following command:
wget -O member_function_with_topic_statistics.cpp https://raw.githubusercontent.com/ros2/examples/humble/rclcpp/topics/minimal_subscriber/member_function_with_topic_statistics.cpp

# Run the subscriber with statistics enabled node:
ros2 run cpp_pubsub listener_with_topic_statistics

# Now run the talker node:
ros2 run cpp_pubsub talker

# While the nodes are running, open a new terminal window. Execute the following command:
ros2 topic list

# Now we can view the statistics data published to this topic with the following command:
ros2 topic echo /statistics

# -------- Run the below code in a new terminal tab --------

# Using Fast DDS Discovery Server as discovery protocol [community-contributed]
fastdds discovery --server-id 0

# Set the environment variable ROS_DISCOVERY_SERVER to the location of the discovery server. (Do not forget to source ROS 2 in every new terminal)
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# Launch the listener node. Use the argument --remap __node:=listener_discovery_server to change the node’s name for this tutorial.
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server

# -------- Run the below code in a new terminal tab --------

# This will create a ROS 2 node, that will automatically create a client for the discovery server and connect to the server created previously to perform discovery, rather than using multicast.
# Set the ROS_DISCOVERY_SERVER environment variable as before so that the node starts a discovery client.
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server

# So far, there is no evidence that this example and the standard talker-listener example are running differently. To clearly demonstrate this, run another node that is not connected to the discovery server. Run a new listener (listening in /chatter topic by default) in a new terminal and check that it is not connected to the talker already running.
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=simple_listener

# To finally verify that everything is running correctly, a new talker can be created using the simple discovery protocol (the default DDS distributed discovery mechanism) for discovery.
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker

# In several terminals, run the following code to establish a communication with redundant servers.
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811
fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888

# --server-id N means server with id N. When referencing the servers with ROS_DISCOVERY_SERVER, server 0 must be in first place and server 1 in second place.
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker
export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

# -------- Run the below code in a new terminal tab --------

# Run the following code to establish a communication with a backed-up server.
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 --backup

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

# Run the first server listening on localhost with the default port of 11811.
fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811

# -------- Run the below code in a new terminal tab --------

# Run the second server listening on localhost using another port, in this case port 11888.
fastdds discovery --server-id 1 --ip-address 127.0.0.1 --port 11888

# Now, run each node in a different terminal. Use ROS_DISCOVERY_SERVER environment variable to decide which server they are connected to. Be aware that the ids must match.

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_1

export ROS_DISCOVERY_SERVER="127.0.0.1:11811;127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_1

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_2

export ROS_DISCOVERY_SERVER=";127.0.0.1:11888"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_2

# Run a talker and a listener that will discover each other through the Server (notice that ROS_DISCOVERY_SERVER configuration is the same as the one in super_client_configuration_file.xml).
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

# Then, instantiate a ROS 2 Daemon using the Super Client configuration (remember to source ROS 2 installation in every new terminal).
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 daemon stop
ros2 daemon start
ros2 topic list
ros2 node info /talker
ros2 topic info /chatter
ros2 topic echo /chatter

# We can also see the Node’s Graph using the ROS 2 tool rqt_graph as follows (you may need to press the refresh button):
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
rqt_graph

# Following the previous configuration, build a simple system with a talker and a listener. First, run a Server:
fastdds discovery -i 0 -l 127.0.0.1 -p 11811

# Then, run the talker and listener in separate terminals:
export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener

export ROS_DISCOVERY_SERVER="127.0.0.1:11811"
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker

# Continue using the ROS 2 CLI with --no-daemon option with the new configuration. New nodes will connect with the existing Server and will know every topic. Exporting ROS_DISCOVERY_SERVER is not needed as the ROS 2 tools will be configured through the FASTRTPS_DEFAULT_PROFILES_FILE.
export FASTRTPS_DEFAULT_PROFILES_FILE=super_client_configuration_file.xml
ros2 topic list --no-daemon
ros2 node info /talker --no-daemon --spin-time 2

# Implementing a custom memory allocator
# The example executable prints the value of the variables. To run the example executable, use:
ros2 run demo_nodes_cpp allocator_tutorial

# or, to run the example with the intra-process pipeline on:
ros2 run demo_nodes_cpp allocator_tutorial intra

# Unlocking the potential of Fast DDS middleware [community-contributed]
# First, create a new package named sync_async_node_example_cpp on a new workspace:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs -- sync_async_node_example_cpp

# Then, add a file named src/sync_async_writer.cpp to the package, with the following content. Note that the synchronous publisher will be publishing on topic sync_topic, while the asynchronous one will be publishing on topic async_topic.
# Finally, ensure you have sourced your setup files and run the node:
source install/setup.bash
ros2 run sync_async_node_example_cpp SyncAsyncWriter

# -------- Run the below code in a new terminal tab --------

# Open two terminals. Do not forget to source the setup files and to set the required environment variables. On the first terminal run the publisher node, and the subscriber node on the other one. You should see that only the /async_topic messages are reaching the subscriber. The /sync_topic subscriber is not receiving the data as it is in a different partition from the corresponding publisher.
# On the first terminal run the service node.
ros2 run sync_async_node_example_cpp ping_service

# You should see the service waiting for requests:
# [INFO] [1612977403.805799037] [ping_server]: Ready to serve

# On the second terminal, run the client node.
ros2 run sync_async_node_example_cpp ping_client

# You should see the client sending the request and receiving the response:
# [INFO] [1612977404.805799037] [ping_client]: Sending request
# [INFO] [1612977404.825473835] [ping_client]: Response received

# At the same time, the output in the server console has been updated:
# [INFO] [1612977403.805799037] [ping_server]: Ready to serve
# [INFO] [1612977404.807314904] [ping_server]: Incoming request
# [INFO] [1612977404.836405125] [ping_server]: Sending back response

# Recording a bag from a node (C++)
# If you’ve installed from Debian packages on Linux, it may be installed by default. If it is not, you can install it using this command.
sudo apt install ros-humble-rosbag2

# -------- Run the below code in a new terminal tab --------

# Open a new terminal and source your ROS 2 installation so that ros2 commands will work. Navigate into the ros2_ws directory created in a previous tutorial. Navigate into the ros2_ws/src directory and create a new package:
ros2 pkg create --build-type ament_cmake bag_recorder_nodes --dependencies example_interfaces rclcpp rosbag2_cpp std_msgs

# Navigate back to the root of your workspace, ros2_ws, and build your new package.
colcon build --packages-select bag_recorder_nodes

# -------- Run the below code in a new terminal tab --------

# Open a new terminal, navigate to ros2_ws, and source the setup files.
source install/setup.bash

# Now run the node:
ros2 run bag_recorder_nodes simple_bag_recorder

# -------- Run the below code in a new terminal tab --------

# Open a second terminal and run the talker example node.
ros2 run demo_nodes_cpp talker

# Terminate both nodes. Then, in one terminal start the listener example node.
ros2 run demo_nodes_cpp listener

# In the other terminal, use ros2 bag to play the bag recorded by your node.
ros2 bag play my_bag

# Navigate back to the root of your workspace, ros2_ws, and build your package.
colcon build --packages-select bag_recorder_nodes

# colcon build --packages-select bag_recorder_nodes
source install/setup.bash

# (If the timed_synthetic_bag directory already exists, you must first delete it before running the node.)

# Now run the node:
ros2 run bag_recorder_nodes data_generator_node

# Wait for 30 seconds or so, then terminate the node with ctrl-c. Next, play back the created bag.
ros2 bag play timed_synthetic_bag

# Open a second terminal and echo the /synthetic topic.
ros2 topic echo /synthetic

# Navigate back to the root of your workspace, ros2_ws, and build your package.
colcon build --packages-select bag_recorder_nodes

# -------- Run the below code in a new terminal tab --------

# Open a terminal, navigate to ros2_ws, and source the setup files.
source install/setup.bash

# Now run the executable:
ros2 run bag_recorder_nodes data_generator_executable

# Now play back the created bag.
ros2 bag play big_synthetic_bag

# -------- Run the below code in a new terminal tab --------

# Open a second terminal and echo the /synthetic topic.
ros2 topic echo /synthetic

# Recording a bag from a node (Python)
# If you’ve installed from Debian packages on Linux, it may be installed by default. If it is not, you can install it using this command.
sudo apt install ros-humble-rosbag2

# Navigate into the ros2_ws/src directory and create a new package:
ros2 pkg create --build-type ament_python bag_recorder_nodes_py --dependencies rclpy rosbag2_py example_interfaces std_msgs

# Navigate back to the root of your workspace, ros2_ws, and build your new package.
colcon build --packages-select bag_recorder_nodes_py

# -------- Run the below code in a new terminal tab --------

# Open a new terminal, navigate to ros2_ws, and source the setup files.
source install/setup.bash

# Now run the node:
ros2 run bag_recorder_nodes_py simple_bag_recorder

# -------- Run the below code in a new terminal tab --------

# Open a second terminal and run the talker example node.
ros2 run demo_nodes_cpp talker

# This will start publishing data on the chatter topic. As the bag-writing node receives this data, it will write it to the my_bag bag. If the my_bag directory already exists, you must first delete it before running the simple_bag_recorder node. This is because rosbag2 will not overwrite existing bags by default, and so the destination directory cannot exist. Terminate both nodes. Then, in one terminal start the listener example node.
ros2 run demo_nodes_cpp listener

# In the other terminal, use ros2 bag to play the bag recorded by your node.
ros2 bag play my_bag

# Navigate back to the root of your workspace, ros2_ws, and build your package.
colcon build --packages-select bag_recorder_nodes_py

# -------- Run the below code in a new terminal tab --------

# Open a new terminal, navigate to ros2_ws, and source the setup files.
source install/setup.bash

# Now run the node:
ros2 run bag_recorder_nodes_py data_generator_node

# Wait for 30 seconds or so, then terminate the node with ctrl-c. Next, play back the created bag.
ros2 bag play timed_synthetic_bag

# -------- Run the below code in a new terminal tab --------

# Open a second terminal and echo the /synthetic topic.
ros2 topic echo /synthetic

# Navigate back to the root of your workspace, ros2_ws, and build your package.
colcon build --packages-select bag_recorder_nodes_py

# -------- Run the below code in a new terminal tab --------

# Open a terminal, navigate to ros2_ws, and source the setup files.
source install/setup.bash

# Now run the executable:
ros2 run bag_recorder_nodes_py data_generator_executable

# Now play back the created bag.
ros2 bag play big_synthetic_bag

# Open a second terminal and echo the /synthetic topic.
ros2 topic echo /synthetic

# Reading from a bag file (C++)
# If you’ve installed from Debian packages on Linux, it may be installed by default. If it is not, you can install it using this command.
sudo apt install ros-humble-rosbag2

# In a new or existing workspace, navigate to the src directory and create a new package:
ros2 pkg create --build-type ament_cmake --license Apache-2.0 bag_reading_cpp --dependencies rclcpp rosbag2_cpp turtlesim

# Navigate back to the root of your workspace and build your new package.
colcon build --packages-select bag_reading_cpp

# Next, source the setup files.
source install/setup.bash

# Now, run the script. Make sure to replace /path/to/setup with the path to your setup bag.
ros2 run bag_reading_cpp simple_bag_reader /path/to/setup

# Installation (Ubuntu)
# You can either install the official released package, or install it from the latest up-to-date sources from Github.
sudo apt-get install ros-humble-webots-ros2

# First source the ROS 2 environment, if not done already.
source /opt/ros/humble/setup.bash

# Setting the WEBOTS_HOME environment variable allows you to start a specific Webots installation.
export WEBOTS_HOME=/usr/local/webots

# If installed from sources, source your ROS 2 workspace, if not done already.
cd ~/ros2_ws
source install/local_setup.bash

# Use the ROS 2 launch command to start demo packages (e.g. webots_ros2_universal_robot).
ros2 launch webots_ros2_universal_robot multirobot_launch.py

# Setting up a robot simulation (Basic)
# Let’s organize the code in a custom ROS 2 package. Create a new package named my_package from the src folder of your ROS 2 workspace. Change the current directory of your terminal to ros2_ws/src and run:
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_robot_driver my_package --dependencies rclpy geometry_msgs webots_ros2_driver

# Let’s add a launch and a worlds folder inside the my_package folder.
cd my_package
mkdir launch
mkdir worlds

# From a terminal in your ROS 2 workspace run:
colcon build
source install/local_setup.bash
ros2 launch my_package robot_launch.py

# -------- Run the below code in a new terminal tab --------

# Then, open a second terminal and send a command with:
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"

# Setting up a robot simulation (Advanced)
# As mentioned in Setting up a robot simulation (Basic), webots_ros2_driver contains plugins to interface most of Webots devices with ROS 2 directly. These plugins can be loaded using the <device> tag in the URDF file of the robot. The reference attribute should match the Webots device name parameter. The list of all existing interfaces and the corresponding parameters can be found on the devices reference page. For available devices that are not configured in the URDF file, the interface will be automatically created and default values will be used for ROS parameters (e.g. update rate, topic name, and frame name).
# In this tutorial, you extended the basic simulation with a obstacle avoider ROS 2 node that publishes velocity commands based on the distance sensor values of the robot.

# The Ros2Supervisor Node
# Here is an example to import a simple Robot named imported_robot:
ros2 service call /Ros2Supervisor/spawn_node_from_string webots_ros2_msgs/srv/SpawnNodeFromString "data: Robot { name \"imported_robot\" }"

# Here is an example on how to remove the imported_robot Robot:
ros2 topic pub --once /Ros2Supervisor/remove_node std_msgs/msg/String "{data: imported_robot}"

# Here is an example on how to start an animation:
ros2 service call /Ros2Supervisor/animation_start_recording webots_ros2_msgs/srv/SetString "{value: " <ABSOLUTE_PATH >/index.html"}"

# The /Ros2Supervisor/animation_stop_recording service is of type webots_ros2_msgs/srv/GetBool and allows to stop the animation.
ros2 service call /Ros2Supervisor/animation_stop_recording webots_ros2_msgs/srv/GetBool "{ask: True}"

# Setting up a Reset Handler
# In this tutorial, you will learn how to implement a reset handler in a robot simulation using Webots. The Webots reset button reverts the world to the initial state and restarts controllers. It is convenient as it quickly resets the simulation, but in the context of ROS 2, robot controllers are not started again making the simulation stop. The reset handler allows you to restart specific nodes or perform additional actions when the reset button in Webots is pressed. This can be useful for scenarios where you need to reset the state of your simulation or restart specific components without completely restarting the complete ROS system.
# In this tutorial, you learned how to implement a reset handler in a robot simulation using Webots. The reset handler allows you to restart specific nodes or perform additional actions when the reset button in Webots is pressed. You explored different approaches based on the complexity of your simulation and the requirements of your nodes.

# Setting up a robot simulation (GAZEBO)
# In this demo you are going to simulate a simple diff drive robot in Gazebo. You are going to use one of the worlds defined in the Gazebo examples called visualize_lidar.sdf. To run this example you should execute the following command in a terminal:
ign gazebo -v 4 -r visualize_lidar.sdf

# When the simulation is running you can check the topics provided by Gazebo with the ign command line tool:
ign topic -l

# Since you have not launched an ROS 2 nodes yet, the output from ros2 topic list should be free of any robot topics:
ros2 topic list

# To be able to communicate our simulation with ROS 2 you need to use a package called ros_gz_bridge. This package provides a network bridge which enables the exchange of messages between ROS 2 and Gazebo Transport. You can install this package by typing:
sudo apt-get install ros-humble-ros-ign-bridge

# At this point you are ready to launch a bridge from ROS to Gazebo. In particular you are going to create a bridge for the topic /model/vehicle_blue/cmd_vel:
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist

# Send a command to the topic using ros2 topic pub
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/Twist "linear: { x: 0.1 }"

# teleop_twist_keyboard package. This node takes keypresses from the keyboard and publishes them as Twist messages. You can install it typing:
sudo apt-get install ros-humble-teleop-twist-keyboard

# The default topic where teleop_twist_keyboard is publishing Twist messages is /cmd_vel but you can remap this topic to make use of the topic used in the bridge:
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel

# The diff drive robot has a lidar. To send the data generated by Gazebo to ROS 2, you need to launch another bridge. In the case the data from the lidar is provided in the Gazebo Transport topic /lidar2, which you are going to remap in the bridge. This topic will be available under the topic /lidar_scan:
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan

# To visualize the data from the lidar in ROS 2 you can use Rviz2:
source /opt/ros/humble/setup.bash
rviz2

# Setting up security
# Before installing from source, you will need to have a recent version openssl (1.0.2g or later) installed:
sudo apt update
sudo apt install libssl-dev

# Fast DDS requires an additional CMake flag to build the security plugins, so the colcon invocation needs to be modified to pass:
colcon build --symlink-install --cmake-args -DSECURITY=ON

# Begin by creating folder to store all the files necessary for this demo:
mkdir ~/sros2_demo

# Use the sros2 utilities to create the keystore. Files in the keystore will be used to enable security for all the participants in the ROS 2 graph.
cd ~/sros2_demo
ros2 security create_keystore demo_keystore

# Once the keystore is created, create keys and certificates for each node with security enabled. For our demo, that includes the talker and listener nodes. This command uses the create_enclave feature which is covered in more detail in the next tutorial.
ros2 security create_enclave demo_keystore /talker_listener/talker
ros2 security create_enclave demo_keystore /talker_listener/listener

# Three environment variables allow the middleware to locate encryption materials and enable (and possibly enforce) security. These and other security-related environment variables are described in the ROS 2 DDS-Security Integration design document.
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Begin the demo by launching the talker node.
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# In another terminal, do the same to launch the listener node. The environment variables in this terminal must be properly set as described in step 4 above.
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener

# Understanding the security keystore
# Use openssl to view this x509 certificate and display it as text:
cd ~/sros2_demo/demo_keys/public
openssl x509 -in ca.cert.pem -text -noout

# The sros2 utilities use elliptic curve cryptograpy rather than RSA for improved security and reduced key size. Use the following command to show details about this elliptic curve private key:
cd ~/sros2_demo/demo_keys/private
openssl ec -in ca.key.pem -text -noout

# Use the following command to validate the S/MIME signature of the governance file:
openssl smime -verify -in governance.p7s -CAfile ../public/permissions_ca.cert.pem

# See if you can answer these questions about the ROS security keystore. Begin with a new terminal session and enable security with the keystore created in the prior tutorial:
export ROS_SECURITY_KEYSTORE=~/sros2_demo/demo_keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
cd ~/sros2_demo/demo_keys/enclaves/talker_listener/listener

# Open permissions.p7s in a text editor. Make a negligible change to the XML content (e.g., add a space or a blank line) and save the file. Launch the listener node:
ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# Ensuring security across machines
# Begin by creating an empty keystore on Bob; the keystore is actually just an empty directory:
ssh Bob
mkdir ~/sros2_demo
exit

# Next copy the keys and certificates for the talker program from Alice to Bob. Since the keys are just text files, we can use scp to copy them.
cd ~/sros2_demo/demo_keystore
scp -r talker USERNAME@Bob:~/sros2_demo/demo_keystore

# Once the environment is set up, run the talker on Bob:
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# Launch the listener on Alice:
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener

# Examining network traffic
# Begin in a new terminal window by installing tcpdump, a command-line tool for capturing and displaying network traffic. Although this tutorial describes tcpdump commands, you can also use Wireshark, a similar graphical tool for capturing and analyzing traffic.
sudo apt update
sudo apt install tcpdump

# Start both the talker and the listener again, each in its own terminal. The security environment variables are not set so security is not enabled for these sessions.
# Disable ROS Security for both terminals
unset ROS_SECURITY_ENABLE

# In terminal 1:
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# In terminal 2:
ros2 run demo_nodes_cpp listener --ros-args --enclave /talker_listener/listener

# The command below uses the -X option to print packet contents, the -i option to listen for packets on any interface, and captures only UDP port 7400 traffic.
sudo tcpdump -X -i any udp port 7400

# Use tcpdump to capture non-discovery RTPS packets by filtering on UDP ports above 7400:
sudo tcpdump -i any -X udp portrange 7401-7500

# Now use tcpdump to capture data packets:
sudo tcpdump -i any -X udp portrange 7401-7500

# Setting access controls
# Begin by making a backup of your permissions files, and open permissions.xml for editing:
cd ~/sros2_demo/demo_keys/enclaves/talker_listener/talker
mv permissions.p7s permissions.p7s~
mv permissions.xml permissions.xml~
vi permissions.xml

# With the updated permissions in place, we can launch the node successfully using the same command used in prior tutorials:
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker

# However, attempting to remap the chatter topic prevents the node from launching (note that this requires the ROS_SECURITY_STRATEGY set to Enforce).
ros2 run demo_nodes_cpp talker --ros-args --enclave /talker_listener/talker \
  --remap chatter:=not_chatter

# Begin by downloading the sros2 repository with the sample policy files:
git clone https://github.com/ros2/sros2.git /tmp/sros2

# In one terminal with security enabled as in previous security tutorials, run the talker demo program:
ros2 run demo_nodes_cpp talker --ros-args -e /talker_listener/talker

# In another terminal do the same with the listener program:
ros2 run demo_nodes_py listener --ros-args -e /talker_listener/listener

# At this point, your talker and listener nodes will be communicating securely using explicit access control lists. However, the following attempt for the listener node to subscribe to a topic other than chatter will fail:
ros2 run demo_nodes_py listener --ros-args --enclave /talker_listener/listener \
  --remap chatter:=not_chatter

# Deployment Guidelines
# In this example, the local host serves as the organization’s system. Let us start by creating a workspace folder:
mkdir ~/security_gd_tutorial
cd ~/security_gd_tutorial

# In order to build a new docker image, a Dockerfile is required. The one proposed for this tutorial can be retrieved with the following command:
# Download the Dockerfile:
wget https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Advanced/Security/resources/deployment_gd/Dockerfile

# Now, build the docker image with the command:
# Build the base image
docker build -t ros2_security/deployment_tutorial --build-arg ROS_DISTRO=humble .

# The compose configuration yaml file can be downloaded with:
# Download the compose file:
wget https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Advanced/Security/resources/deployment_gd/compose.deployment.yaml

# In the same working directory ~/security_gd_tutorial, run:
# Start the example:
docker compose -f compose.deployment.yaml up

# While having the containers running that simulate the two remote devices for this tutorial, attach to each of them by opening two different terminals and enter:
# Terminal 1:
docker exec -it tutorial-listener-1 bash
cd keystore
tree

# Terminal 2:
docker exec -it tutorial-talker-1 bash
cd keystore
tree
