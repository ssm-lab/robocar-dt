# Digital Twin of an Autonomous Scale Vehicle  
Author: Adwita Kashyap  
Supervisor: Dr. Istvan David
## Introduction
This research project aims to build a digital twin of a sensor-equipped robot car using real-time data streaming. The digital twin is a virtual representation of the physical system's sensor outputs while also enabling control of the system. In this research project, we offload computation-intensive tasks such as object detection to the digital twin for faster processing.
## Software
- Language: Python
- ROS2 distribution: Humble
- Object detection: YOLOv11 custom trained model
- Communication Frameworks:
    - ZeroMQ: Used for communication of controls between the physical twin and the digital twin
    - Image ZeroMQ: Used for communication of sensor data between the physical twin and the digital twin
## Hardware
- Robot platform: Hiwonder JetAcker
- Chassis: Ackerman
- Onboard computer: Raspberry Pi 5 (8GB)
- Sensors:
    - SLAMTEC A1 LiDAR
    - Orbbec Astra Pro Plus (RGB + depth camera)
## Setup
This project requires code to be run on both the vehicle's onboard computer and the user's remote system. Below is a breakdown of the relevant libraries and files.
### Physical Twin:
Install requirements listed in requirements/car.txt
Put the following files, located in main, on the physical twin's computer: 
- CarController.py
- CarPublisher.py
- CarRoutePlanner.py
### Digital Twin:
Install requirements via ```pip install -r requirements/user.txt```.
Run the following file, located in main, on the digital twin:
- UserObjectDetector.py




