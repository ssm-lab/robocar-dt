import time
from Publisher import Publisher

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Car publisher: 
- Runs on the car's raspberry pi
- Sends data to UserSubscriber.py
"""

class CarPublisher(Publisher):

    def __init__(self):
        super().__init__("5557")

    def publish(self):
        running = True
        i = 1
        while running:
            command = i ## Send actual data
            self._publisher.send_json(command)
            i+=1
            time.sleep(2)
        
if __name__ == '__main__':
    carPublisher = CarPublisher()
    carPublisher.publish()