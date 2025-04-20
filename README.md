# Robotic-arm-using-Computer-Vision-for-Object-sorting-based-on-shape-and-colour
The robotic arm uses computer vision to classify and sort objects based on their shapes and colours
# Project Overview
This project focuses on the simulation and control of an industrial robotic arm (ABB IRB 140) to perform pick-and-place operations based on object color and shape. The system uses MATLAB for image processing and CoppeliaSim for robot simulation, creating an integrated environment for real-time object detection, classification, and robotic manipulation.

The project demonstrates a cost-effective, vision-based automated sorting system suitable for industrial environments, especially in manufacturing and logistics.

# Features
Vision-based object classification using MATLAB

Real-time object sorting based on color (Red, Green, Blue) and shape (Cube, Cuboid, Cylinder)

Full simulation of ABB IRB 140 robotic arm in CoppeliaSim

Object recognition using HSV color space and Sobel/Laplacian edge detection

Robotic motion control using Inverse Kinematics

Path planning using A* algorithm

Simulated conveyor belt with randomly generated objects

# Tools and Technologies
MATLAB (Image Processing Toolbox)

CoppeliaSim (formerly V-REP) – Robotic simulation platform

Lua Scripting – Object generation and robot control in simulation

Remote API (MATLAB ↔ CoppeliaSim) – Real-time data exchange

ABB IRB 140 – 6-DOF industrial robot model used in simulation

 # How to Run
Prerequisites
MATLAB R2020 or later (with Image Processing Toolbox)

CoppeliaSim (Edu/Pro version)

Lua (included in CoppeliaSim)

Basic understanding of kinematics and image processing

Setup Instructions
Launch CoppeliaSim:

Open scene.ttt from the CoppeliaSim/ directory.

Start the simulation.

Start MATLAB Script:

Navigate to the MATLAB/ directory.

Run image_processing.m or the main script that starts communication and classification.

Watch the Simulation:

Objects will be created on the conveyor.

When an object reaches the vision sensor, MATLAB processes the image, classifies it, and instructs the robotic arm to sort it into the correct bin.
