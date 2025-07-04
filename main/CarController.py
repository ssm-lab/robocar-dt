import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
Car controller: 
- Runs on the car's raspberry pi
- Turns control commands received by CarSubscriber into Twist messages for ROS
- Allows for the control commands to actually move the car
"""

class CarController(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('car_controller')
        self.publisher = self.create_publisher(Twist, '/controller/cmd_vel', 10)

    def move(self, speed = 0.1, angSpeed = 0.0, duration = 1.0):
        try:
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = angSpeed
            self.publisher.publish(msg)
            
            logMsg = "Speed: " + str(speed) + " m/s. Angular speed: " + str(angSpeed) + " rad/s. Duration: " + str(duration) + " s."
            self.get_logger().info(logMsg)
            time.sleep(duration)

        except Exception as e:
            errorMsg = "Stopping car, something went wrong: " + str(e)
            self.get_logger().error(errorMsg)
        
        finally:
            self.stop()

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def end(self):
        self.destroy_node()
        rclpy.shutdown()

def main():
    controller = CarController()
    time.sleep(3)
    controller.move(speed = 0.3, angSpeed = -0.4, duration = 2.0)
    controller.end()

if __name__ == '__main__':
    main()

