from Subscriber import Subscriber

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
User subscriber: 
- Runs on the user's device
- Receives data from CarPublisher.py
"""

class UserSubscriber(Subscriber):

    def __init__(self):
        super().__init__("192.168.149.1","5557")
        
    def subscriberAction(self):
        receviedMessage = self._subscriber.recv()
        print(receviedMessage) ## Process received data

if __name__ == '__main__':
    subscriber = UserSubscriber()
    subscriber.subscribe()