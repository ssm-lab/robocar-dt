import rclpy
import imagezmq
#from Publisher import Publisher
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node


__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Car publisher: 
- Runs on the car's raspberry pi
- Sends data to UserSubscriber.py
"""

class CarPublisher(Node):

    def __init__(self):
        #super().__init__("5557") # image publisher setup

        super().__init__("imageSender")
        self.pub = imagezmq.ImageSender(connect_to='tcp://*:5557', REQ_REP = False) # Initialize image sender as pub/sub
        self.bridge = CvBridge() # bridge to convert image from ROS to openCV readable
        self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.publish, 10) # ROS topic subscriber to get image

    def publish(self, image):
        '''running = True
        i = 1
        while running:
            command = i ## Send actual data
            self._publisher.send_json(command)
            i+=1
            time.sleep(2)'''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            self.pub.send_image("hello", cv_image)
            print("message sent!")

def main():
    rclpy.init()
    carPublisher = CarPublisher()
    rclpy.spin(carPublisher) 
    carPublisher.destroy_node()
    rclpy.shutdown()

        
if __name__ == '__main__':
    main()
    #carPublisher.publish()