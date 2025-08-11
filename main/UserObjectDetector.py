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
- Receives images from CarPublisher.py (pub/sub)
- Runs object detection using YOLO on received images
- Sends coordinates of bounding box to CarRoutePlanner.py (req/rep)
"""

class ObjectDetector():

    def __init__(self):
        self.imageHub = imagezmq.ImageHub(open_port='tcp://192.168.149.1:5557', REQ_REP = False)
        self.ctx = zmq.Context()
        self.req = self.ctx.socket(zmq.REQ)
        self.req.bind("tcp://*:5558")
        self.poller = zmq.Poller()
        self.poller.register(self.req, zmq.POLLIN)
        self.model = YOLO("C:/Users/adwit/runs/detect/train9/weights/best.pt")
        
    def processFrame(self, buffer):
        """Receives buffer frame, converts to openCV format, runs object detection, and displays the frame.
        Returns the number of frames to skip to avoid lag and coordinates of bounding box"""
        fps = 30 # Set on the car's end
        allowedTime = 1/fps # Time allowed for processing each frame
        startTime = time.time()

        # Display processed frame
        buffer = bytes(buffer)
        image = cv2.imdecode(np.frombuffer(buffer, dtype=np.uint8), cv2.IMREAD_COLOR)
        results = self.model.predict(image, conf = 0.33)[0]
        image = results.plot()
        cv2.imshow("live", image)

        # Get bounding box data
        try:
            confScores = results.boxes.conf.tolist()
            maxConf = max(confScores)
            maxIndex = confScores.index(maxConf)
            bbox = results.boxes.xyxy[maxIndex].tolist() 
        except:
            # In case of no detections
            bbox = None
        
        # Calculate processing time
        elapsedTime = time.time() - startTime

        # Find number of frames to skip based on processing time
        if elapsedTime > allowedTime:
            framesToSkip = ceil((elapsedTime - allowedTime) * fps)
        else:
            framesToSkip = 0

        return framesToSkip, bbox

    def communicate(self):
        running = True
        firstFrame = True
        framesToSkip = 0

        while running:
            # Receives frames- drops without further processing if there is a lag
            for i in range (framesToSkip + 1):
                _, buffer = self.imageHub.recv_jpg() 

            framesToSkip, bbox = self.processFrame(buffer)

            # Send bbox to car if car is ready
            if firstFrame:
                self.req.send_json(bbox)
                print(f"sent: {bbox}")
                firstFrame = False

            items = dict(self.poller.poll(1))
            if self.req in items:
                reply = self.req.recv_string()
                print(f"REPLY FROM CAR: {reply}")
                if reply == "target reached":
                    running = False
                else:
                    self.req.send_json(bbox)
                    print(f"sent: {bbox}")

            # Press any key to exit
            if cv2.waitKey(1) > -1:
                running = False
                
        cv2.destroyAllWindows()


if __name__ == '__main__':
    objectDetector = ObjectDetector()
    objectDetector.communicate()