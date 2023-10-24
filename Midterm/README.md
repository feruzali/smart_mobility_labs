# Dynamic Fleet Management with ROS2 (Python)

## Project Overview

This repository contains the implementation of a dynamic fleet management system using ROS2 (Robot Operating System 2) in Python. The primary objective of the project is to efficiently allocate and route vehicles in a smart mobility service. The implementation includes both the Action Server and Action Client components, complemented by a professional Command Line Interface (CLI) for user interaction.

## Objectives

The main objectives for this assignment were as follows:

1. Gain practical experience in developing ROS2 applications with Python.
2. Create a functional ROS2 project for dynamic fleet management.
3. Learn how to use GitHub for version control.
4. Practice proper documentation and reporting.

## Task Descriptions

### Task 1: Defined ROS2 Action

In this step, a ROS2 Action file named `fleet_management.action` was created. This file defined the Action Goal, Result, and Feedback messages. The Action Goal included an integer field for the fleet size, the Action Result consisted of an array of strings representing vehicle routes, and the Action Feedback contained a float indicating the completion percentage.

### Task 2: Implemented the Action Server

For this task, a Python script named `fleet_management_server_cli.py` was developed to serve as the Action Server. This server is responsible for receiving fleet size requests, executing fleet management logic (e.g., allocation and routing), and returning the calculated routes as the Action Result. Proper error handling and logging were incorporated for a robust implementation.

### Task 3: Developed the Action Client CLI

In this step, a Python script named `fleet_management_client_cli.py` was created to function as the Action Client. This client allows users to request fleet management tasks by specifying the desired fleet size. It sends the request to the server, receives the routes in response, and presents them to the user.

### Task 4: Crafted a Professional CLI

Here, a user-friendly Command Line Interface (CLI) was designed using the `click` library. The CLI provides an option for users to allocate and route vehicles by specifying the fleet size. Internally, it calls the Action Client CLI to execute the task seamlessly.

### Task 5: Tested with Scenarios

Two distinct scenarios, each with real-world data, were created to rigorously test the application. For each scenario, details such as the fleet size and expected routes were provided.

### Task 6: Documented the Project

Thorough documentation was provided to ensure clarity and ease of understanding for future developers and users. This included documentation of code, usage, and overall project structure.

## Strategy

The use of Git for version control and proper documentation ensured a functional and well-documented ROS2 project. Following a systematic approach, the project successfully achieved its objectives and provides a scalable foundation for future enhancements and collaborations.

**Output Results:**

- **Client:**

![client](https://github.com/feruzali/smart_mobility_labs/assets/50808895/66567daf-6689-4ec2-bf12-f252be176de7)


- **Server:**

![server](https://github.com/feruzali/smart_mobility_labs/assets/50808895/c937a4ad-d791-4cfb-af09-09d57c6058b0)

