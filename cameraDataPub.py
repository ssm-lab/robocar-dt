# This program will run on the pi to subscribe to the appropriate topic to get camera data and use zmq to send that data to another computer

import rclpy
import zmq
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        context = zmq.Context()
        pub = context.socket(zmq.PUB)
        pub.bind("tcp://*:5555")

        self.sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.callback, 10)

    def callback(self, msg):
        self.sendImage(msg.data)

    def sendImage(self, msg):
        
        self.pub.send_string("cat")
        print("Message sent")
        

def main():
    rclpy.init()
    node = ImageSubscriber()

    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
