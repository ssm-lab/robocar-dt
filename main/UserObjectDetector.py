import cv2
import zmq
import time
import imagezmq
import numpy as np
from math import ceil
from ultralytics import YOLO

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
User subscriber: 
- Runs on the user's device
- Receives images from CarPublisher.py
- Runs object detection using YOLO on received images
"""

class UserSubscriber():

    def __init__(self):
        self.imageHub = imagezmq.ImageHub(open_port='tcp://192.168.149.1:5557', REQ_REP = False)
        
        self.ctx = zmq.Context()
        self.req = self.ctx.socket(zmq.REQ)
        self.req.bind("tcp://*:5558")
        self.poller = zmq.Poller()
        self.poller.register(self.req, zmq.POLLIN)
        self.model = YOLO("C:/Users/adwit/runs/detect/train9/weights/best.pt")
        
    def processFrame(self, buffer):
        
        fps = 30 ## from hardware stats
        allowedTime = 1/fps # Time allowed for processing each frame
        
        # Display processed frame + get processing time
        startTime = time.time()

        buffer = bytes(buffer)
        image = cv2.imdecode(np.frombuffer(buffer, dtype=np.uint8), cv2.IMREAD_COLOR)
        results = self.model.predict(image)[0]
        image = results.plot()
        cv2.imshow("live", image)

        elapsedTime = time.time() - startTime

        # Find number of frames to skip
        if elapsedTime > allowedTime:
            framesToSkip = ceil((elapsedTime - allowedTime) * fps)
        else:
            framesToSkip = 0

        return framesToSkip

    def subscribe(self):
        running = True
        firstFrame = True
        framesToSkip = 0
        command = "dummy command"

        while running:
            # receives frames- drops without further processing if there is a lag
            for i in range (framesToSkip + 1):
                _, buffer = self.imageHub.recv_jpg() 

            framesToSkip = self.processFrame(buffer)

            if firstFrame:
                self.req.send_string(command)
                print("command sent")
                firstFrame = False

            items = dict(self.poller.poll(1))
            if self.req in items:
                reply = self.req.recv_string()
                print(reply)
                self.req.send_string(command)
                print("command sent") 

            # Press any key to exit
            if cv2.waitKey(1) > -1:
                running = False
                
        cv2.destroyAllWindows()


if __name__ == '__main__':
    subscriber = UserSubscriber()
    subscriber.subscribe()