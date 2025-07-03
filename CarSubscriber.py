from Subscriber import Subscriber

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Car subscriber: 
- Runs on the car's raspberry pi
- Receives car control commands from UserPublisher.py
"""

class CarSubscriber(Subscriber):

    def __init__(self):
        super().__init__("192.168.149.150","5558")
        
    def subscriberAction(self):
        receivedMessage = self._subscriber.recv_json()
        print(receivedMessage) ## Do other tasks with received message


if __name__ == '__main__':
    subscriber = CarSubscriber()
    subscriber.subscribe()