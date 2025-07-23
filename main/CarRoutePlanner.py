import zmq
import time
from Subscriber import Subscriber
from CarController import CarController

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Car subscriber: 
- Runs on the car's raspberry pi
- Receives car control commands from UserPublisher.py
"""

class CarSubscriber():

    def __init__(self):
        #super().__init__("192.168.149.150","5558")
        self.ctx = zmq.Context()
        self.rep = self.ctx.socket(zmq.REP)
        self.rep.connect("tcp://192.168.149.150:5558")
        self.poller = zmq.Poller()
        self.poller.register(self.rep, zmq.POLLIN)
        
    def subscribe(self):
        controller = CarController()
        running = True
        reply = "received"
        while running:
            #receivedMessage = self._subscriber.recv_json()
            receivedMessage = self.rep.recv_string()

            if 'q' in receivedMessage:
                controller.end()
                running = False
            else:
                #controller.move(speed = receivedMessage[0], angSpeed = receivedMessage[1], duration = receivedMessage[2])
                print(receivedMessage)
                time.sleep(0.5)
            self.rep.send_string(reply)


if __name__ == '__main__':
    subscriber = CarSubscriber()
    subscriber.subscribe()