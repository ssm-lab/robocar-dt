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

        self.sub = self.create_subscription(Image, '/depth_cam/depth/image_raw', self.subscribe, 1)
        self.bridge = CvBridge()

        self.controller = CarController()

    def subscribe(self, msg):
        items = dict(self.poller.poll(1))

        if self.rep in items:
            bbox = self.rep.recv_json()
            image = self.bridge.imgmsg_to_cv2(msg)
            reply = self.navigate(image, bbox)
            self.rep.send_string(reply)
        
    def navigate(self, image, bbox):
        cameraFOV = 58.4 # Camera's horizontal field of view (degrees)
        frameWidth = 640 # pixels
        linearSpeed = 0.3

        depthSum = 0 # in mm
        numOfElements = 0
        xCentre = (bbox[0] + bbox[2])/2
        angle = math.radians((xCentre - frameWidth/2) * cameraFOV/frameWidth)

        if angle > 0:
            rightTurn = True
        else:
            angle = abs(angle)
            rightTurn = False

        for x in range(int(bbox[0]), int(bbox[2])):
            for y in range (int(bbox[1]), int(bbox[3])):
                if image[y][x] != 0:
                    depthSum += image[y][x]
                    numOfElements += 1

        # If depth camera cannot sense cube
        if depthSum == 0:
            distance = 0.5
            reply = "searching"
            print(reply)
        else:
            distance = depthSum/ (numOfElements*1000) - 0.2 #in m
            reply = "target reached"

        turnRadius = distance/(2*math.sin(angle))
        travelTime = 2*turnRadius*angle/linearSpeed
        angularSpeed = linearSpeed/turnRadius
        
        if rightTurn:
            angularSpeed = -angularSpeed
            
        self.controller.move(linearSpeed, angularSpeed, travelTime)

        if reply == "target reached":
            self.controller.stop()

        return (reply)
    
def main():
    rclpy.init()
    planner = CarRoutePlanner()
    rclpy.spin(planner) 
    planner.controller.end()

if __name__ == '__main__':
    main()