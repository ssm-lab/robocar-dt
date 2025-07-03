import zmq

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Parent Publisher class. Can be implemented on either the raspberry pi or the user's device.
"""

class Publisher():

    def __init__(self, port):
        self.ctx = zmq.Context()
        self._publisher = self.ctx.socket(zmq.PUB)
        self._publisher.bind(f"tcp://*:{port}")

    def publish(self):
        raise NotImplementedError