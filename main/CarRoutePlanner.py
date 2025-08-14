import zmq
import math
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from CarController import CarController

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Car route planner: 
- Runs on the car's raspberry pi
- Receives coordinates of bounding box containing target object
- Plans route and navigates to the given location
- Explores surroundings if target object is not detected
"""

class CarRoutePlanner(Node):

    def __init__(self):
        super().__init__("planner")

        self.ctx = zmq.Context()
        self.rep = self.ctx.socket(zmq.REP)
        self.rep.connect("tcp://192.168.149.150:5558")
        self.poller = zmq.Poller()
        self.poller.register(self.rep, zmq.POLLIN)

        self.sub = self.create_subscription(Image, '/depth_cam/depth/image_raw', self.communicate, 1)
        self.bridge = CvBridge()

        self.controller = CarController()
        self.exploreRadius = 0.25
        self.exploreCount = 0
        self.sawCube = False

    def communicate(self, msg):
        '''Receives bounding box data from digital twin. Sends back a reply with the status of the car's search for target'''
        items = dict(self.poller.poll(1))

        if self.rep in items:
            bbox = self.rep.recv_json()
            if bbox is None:
                reply = self.explore()
                self.sawCube = False
            else:
                self.exploreRadius = 0.5
                self.exploreCount = 0
                image = self.bridge.imgmsg_to_cv2(msg)
                reply = self.navigate(image, bbox)
                self.sawCube = True
            self.rep.send_string(reply)
        
    def navigate(self, image, bbox):
        '''Plans and executes route when target object is detected. Returns status of the vehicle's search'''
        cameraFOV = 58.4 # Camera's horizontal field of view (degrees)
        frameWidth = 640 # pixels
        linearSpeed = 0.3

        # Angle between car and object
        xCentre = (bbox[0] + bbox[2])/2
        angle = math.radians((xCentre - frameWidth/2) * cameraFOV/frameWidth)

        # Distance between car an object
        depthSum = 0 # in mm
        numOfElements = 0

        for x in range(int(bbox[0]), int(bbox[2])):
            for y in range (int(bbox[1]), int(bbox[3])):
                if image[y][x] != 0:
                    depthSum += image[y][x]
                    numOfElements += 1

        # If depth camera cannot sense cube
        if depthSum == 0:
            distance = 0.5
            reply = "navigating without depth"
        else:
            distance = depthSum/ (numOfElements*1000) - 0.2 #in m
            reply = "target reached"
        
        # Calculate movement command
        turnRadius = distance/(2*math.sin(angle))
        travelTime = abs(2*turnRadius*angle/linearSpeed)
        angularSpeed = -linearSpeed/turnRadius

        # Move
        self.controller.move(linearSpeed, angularSpeed, travelTime)
        self.controller.stop()

        return (reply)

    def explore(self):
        '''Runs when there are no object detections.'''
        if self.sawCube:
            reply = "target reached"
            self.controller.stop()

        else: 
            linearSpeed = 0.1
            angularSpeed = linearSpeed/self.exploreRadius
            travelTime = 0.2
            reply = "exploring"
            
            self.controller.move(linearSpeed, angularSpeed, travelTime)
            
            self.exploreCount += 1

            ## If completed circle while exploring, increase radius
            if self.exploreCount >= 2*math.pi*self.exploreRadius/(linearSpeed*travelTime):
                self.exploreRadius += 0.25
                self.exploreCount = 0

        return reply
        

def main():
    rclpy.init()
    planner = CarRoutePlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        print(" Shutting down.")

if __name__ == '__main__':
    main()