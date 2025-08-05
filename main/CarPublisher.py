import cv2
import rclpy
import imagezmq
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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
        super().__init__("imageSender")

        self.pub = imagezmq.ImageSender(connect_to='tcp://*:5557', REQ_REP = False) # Initialize image sender as pub/sub
        self.bridge = CvBridge() # bridge to convert image from ROS to openCV readable
        self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.publish, 10) # ROS topic subscriber to get image

    def publish(self, image):
        # convert image to jpg format
        cvImage = self.bridge.imgmsg_to_cv2(image, "bgr8")
        _, jpgImage = cv2.imencode('.jpg', cvImage)

        # publish information
        self.pub.send_jpg("", jpgImage.tobytes())

def main():
    rclpy.init()
    carPublisher = CarPublisher()
    try: 
        rclpy.spin(carPublisher) 
    except KeyboardInterrupt:
        print(" Program exited by user.")
    finally:
        carPublisher.destroy_node()

        
if __name__ == '__main__':
    main()