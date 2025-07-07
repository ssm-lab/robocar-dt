import imagezmq
import cv2

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
            print("listening")
            msg, image = self.imageHub.recv_image()
            
            cv2.imshow("live", image)
            print(cv2.mean(image))

            if cv2.waitKey(1) > -1:
                running = False
                print("Stopping")
            
        print("done")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    subscriber = UserSubscriber()
    subscriber.subscribe()