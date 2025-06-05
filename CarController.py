import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.publisher = self.create_publisher(Twist, '/controller/cmd_vel')

    def drive(self, speed = 0.1, duration = 2.0):
        
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        if speed>0:
            self.get_logger().info('Moving forward...')
        elif speed<0:
            self.get_logger().info("Moving backwards...")
        time.sleep(duration)

        # Stop after moving
        self.stop()

    def turn(self, speed = 0.1, angSpeed = 0.1, duration = 1.0):
        '''left = angSpeed > 0, right = angSpeed < 0'''
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = angSpeed
        self.publisher.publish(msg)

        if angSpeed > 0:
            self.get_logger().info("Turning left...")
        elif angSpeed < 0:
            self.get_logger().info("Turning right...")

        time.sleep(duration)

        self.stop()

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Stopping...')

def main(): ## 
    rclpy.init() ###
    controller = CarController()

    controller.drive()
    controller.turn()
    controller.drive(speed=-0.1)
    controller.stop()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
