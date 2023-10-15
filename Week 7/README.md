# Robot Navigation Towards Goal using ROS ActionLib

## Application Description

**Introduction:**

The Robot Navigation Towards Goal using ROS ActionLib is a system designed for enabling robots to navigate through their environment autonomously to reach a specified goal. The communication between the client and server is facilitated through the ROS Action Protocol, implemented using ROS Actionlib. This system defines three fundamental actions: goal, feedback, and result, establishing a standardized interface for communication.

**Python Files Description:**

1. **Navigate2D.action:**
   - Defines the type and format of the goal, result, and feedback topics for the action.
   - Specifies the goal as a Point using geometry_msgs, containing the 3D position of a point.
   - Defines feedback and result with the float message type.

2. **robot_point_publisher.py:**
   - Prompts the user for the current point of the robot with three-axis coordinates.
   - Publishes this information on the 'robot/point' topic.

3. **action_client.py:**
   - Defines action messages specifying goal, feedback, and result.
   - Sends the user's desired location to the action server to navigate the robot towards its goal.
   - Receives feedback and result as a response from the server.

4. **action_server.py:**
   - Defines action messages for communication with the client.
   - Subscribes to the 'robot/point' topic to obtain the robot's goal.
   - Calculates the distance between the current location and the goal.
   - Provides feedback to the client about the remaining distance during navigation.
   - Stops execution and returns the time taken when the robot reaches the defined threshold.

**Requirements:**

1. **Linux OS - Ubuntu 20.04:**
   - The system is developed and tested on the Ubuntu 20.04 operating system.

2. **Python 3.7 or above:**
   - Requires Python version 3.7 or higher for proper execution.

## Interaction Diagram

The interaction diagram illustrates the flow of communication between the client and server components:

![diagram](https://github.com/feruzali/smart_mobility_labs/assets/50808895/abb0d291-8482-4293-9a67-ee56e25c6e8c)

This refined diagram provides a more detailed sequence of interactions:

1. The Client sends a goal request to the Server.
2. The Server receives the goal and acknowledges the request.
3. The Server calculates the distance and initiates navigation.
4. The Server provides feedback to the client during navigation.
5. The Server checks if the robot has reached the defined threshold.
6. If the threshold is reached, the Server stops navigation and returns the result and navigation time to the client.

This breakdown gives a clearer understanding of the steps involved in the communication flow between the Action Client and Action Server during the robot navigation process.
