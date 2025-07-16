import cv2
import time
import imagezmq
import numpy as np
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
        self.startTime = time.time()
        
    def subscribe(self):
        running = True
        framesToSkip = 0
        fps = 30 ## from hardware stats
        allowedTime = 1/fps # Time allowed for processing each frame

        path = "C:/Users/adwit/runs/detect/train2/weights/best.pt"
        model = YOLO(path)

        while running:
            for i in range (framesToSkip + 1):
                timestamp, buffer = self.imageHub.recv_jpg() 

            startTime = time.time()

            buffer = bytes(buffer)
            image = cv2.imdecode(np.frombuffer(buffer, dtype=np.uint8), cv2.IMREAD_COLOR)
            results = model.predict(image)[0]
            image = results.plot()
            cv2.imshow("live", image)

            elapsedTime = time.time() - startTime

            ## Find number of frames to skip
            if elapsedTime> allowedTime:
                framesToSkip = int((elapsedTime - allowedTime) * fps) 
            else:
                framesToSkip = 0

            # Press any key to exit
            if cv2.waitKey(1) > -1:
                running = False
                
        cv2.destroyAllWindows()


if __name__ == '__main__':
    subscriber = UserSubscriber()
    subscriber.subscribe()