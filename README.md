# Digital Twin of Raspberry Pi-Based Robot Car
Status: In progress  
Author: Adwita Kashyap  
Supervisor: Dr. Istvan David
## Introduction
This research project aims to build a digital twin of a sensor-equipped robot car using real-time data streaming. The digital twin is a virtual replica that reflects the physical car’s behavior and sensor outputs, while also enabling control of the car based on that information.  
A key feature of this system is its split design, which allows the robot to run ROS 2, while the user interacts with it from another computer without needing to install ROS. The two systems communicate in real time using ZeroMQ.
## Software
- Language: Python
- Communication Frameworks:
    - ROS 2 Humble: Used for intra-robot communication
    - ZeroMQ: Used for communication between the robot car and the user’s machine
## Hardware
- Robot platform: Hiwonder JetAcker
- Chassis: Ackermann
- Onboard computer: Raspberry Pi 5 (8GB)
- Sensors:
    - SLAMTEC A1 LiDAR
    - Orbbec Astra Pro Plus Depth Camera
## Setup
This project requires code to be run on both the car's onboard computer and the user's remote system. Below is a breakdown of the relevant files.
### Files to run on the car:
- Publisher.py
- Subscriber.py
- CarController.py
- CarPublisher.py
- CarSubscriber.py
### Files to run on the user’s computer:
- Publisher.py
- Subscriber.py
- UserPublisher.py
- UserSubscriber.py




