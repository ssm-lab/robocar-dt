import zmq
import math
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from CarController import CarController

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Car subscriber: 
- Runs on the car's raspberry pi
- Receives coordinates of bounding box
- Plans route and navigates to the given location
"""

class CarRoutePlanner(Node):

    def __init__(self):
        super().__init__("planner")

        self.ctx = zmq.Context()
        self.rep = self.ctx.socket(zmq.REP)
        self.rep.connect("tcp://192.168.149.150:5558")
        self.poller = zmq.Poller()
        self.poller.register(self.rep, zmq.POLLIN)

        self.sub = self.create_subscription(Image, '/depth_cam/depth/image_raw', self.subscribe, 1) # ROS topic subscriber to get image
        self.bridge = CvBridge()

        self.controller = CarController()

    def subscribe(self, msg):
        items = dict(self.poller.poll(1))

        if self.rep in items:
            bbox = self.rep.recv_json()
            print(f"bbox: {bbox}") 
            reply = self.plan(msg, bbox)
            self.rep.send_string(reply)
        
    def plan(self, msg, bbox):
        cameraFOV = 58.4 # Camera's horizontal field of view (degrees)
        frameWidth = 640 # pixels
        xCentre = (bbox[0] + bbox[2])/2
        linearSpeed = 0.3

        angle = math.radians((xCentre - frameWidth/2) * cameraFOV/frameWidth)
        if angle > 0:
            rightTurn = True
        else:
            angle = abs(angle)
            rightTurn = False

        print(f"angle: {angle}. Right turn: {rightTurn}")

        depthSum = 0 # in mm
        numOfElements = (int(bbox[2]) - int(bbox[0])) * (int(bbox[3] - int(bbox[1])))

        image = self.bridge.imgmsg_to_cv2(msg)
        print(f"cv image shape (bridge): {image.shape}")

        for x in range(int(bbox[0]), int(bbox[2])):
            for y in range (int(bbox[1]), int(bbox[3])):
                print(f"{image[y][x]}, ", end="")
                depthSum += image[y][x]
            print()

        averageDepth = depthSum/ (numOfElements*1000) #in m
        turnRadius = averageDepth/(2*math.sin(angle))
        angularSpeed = linearSpeed/turnRadius

        print(f"average depth: {averageDepth}")
        print(f"turn radius: {turnRadius}")
        
        
        if rightTurn:
            angularSpeed = -angularSpeed
        print(f"angular: {angularSpeed}") #debug

        travelTime = 2*turnRadius*angle/linearSpeed
        print(f"Time: {travelTime}") #debug
        
        #self.controller.move(linearSpeed, angularSpeed, travelTime)
        print(f"Command (sending paused for now): Linear = {linearSpeed}. Angular = {angularSpeed}. Time = {travelTime}")

        return ("target reached!")
    
def main():
    rclpy.init()
    planner = CarRoutePlanner()
    rclpy.spin(planner) 
    planner.controller.end()

if __name__ == '__main__':
    main()