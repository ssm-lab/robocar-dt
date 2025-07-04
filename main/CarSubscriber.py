from Subscriber import Subscriber
from CarController import CarController

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
        
    def subscribe(self):
        controller = CarController()
        running = True
        while running:
            receivedMessage = self._subscriber.recv_json()

            if 'q' in receivedMessage:
                controller.end()
                running = False
            else:
                controller.move(speed = receivedMessage[0], angSpeed = receivedMessage[1], duration = receivedMessage[2])


if __name__ == '__main__':
    subscriber = CarSubscriber()
    subscriber.subscribe()