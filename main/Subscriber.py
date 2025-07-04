import zmq
import time
import logging

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Parent Subscriber class. Can be implemented on either the raspberry pi or the user's device.
"""

class Subscriber():

    def __init__(self, publisherIP, port):
        self.ctx = zmq.Context()
        self._subscriber = self.ctx.socket(zmq.SUB)
        #print("Starting sub")
        self._subscriber.subscribe("")
        self._subscriber.connect(f"tcp://{publisherIP}:{port}")
        print("connected")
    
    def subscribe(self):
        raise NotImplementedError