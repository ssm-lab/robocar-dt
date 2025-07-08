import imagezmq
import cv2
import numpy as np

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
User subscriber: 
- Runs on the user's device
- Receives data from CarPublisher.py
"""

class UserSubscriber():

    def __init__(self):
        self.imageHub = imagezmq.ImageHub(open_port='tcp://192.168.149.1:5557', REQ_REP = False)
        
    def subscribe(self):
        running = True
        while running:
            title, buffer = self.imageHub.recv_jpg() 
            buffer = bytes(buffer)
            image = cv2.imdecode(np.frombuffer(buffer, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow(title, image)

            # Press any key to exit
            if cv2.waitKey(1) > -1:
                running = False
            
        print("done")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    subscriber = UserSubscriber()
    subscriber.subscribe()