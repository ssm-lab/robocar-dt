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
        print("Starting sub")
        self._subscriber.subscribe("")
        self._subscriber.connect(f"tcp://{publisherIP}:{port}")

        self._poller = zmq.Poller()
        self._poller.register(self._subscriber, zmq.POLLIN)
    
    def subscribe(self):
        logging.debug("Receiving messages")
        
        alarm = time.time() + 1.
        while True:
            tickless = 1000 * max(0, alarm - time.time())
            try:
                items = dict(self._poller.poll(tickless))
            except:
                break  # Interrupted
    
            if self._subscriber in items:
                self.subscriberAction()
    
        logging.debug("Interrupted")

    def subscriberAction(self):
        raise NotImplementedError