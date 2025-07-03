import zmq
import CarController
import rclpy

## This version of the subscriber is meant to be run on the pi
context = zmq.Context()
sub = context.socket(zmq.SUB)
sub.connect("tcp://192.168.149.150:5555")
sub.setsockopt_string(zmq.SUBSCRIBE, "")
print("Starting subscriber...")

rclpy.init()
controller = CarController.CarController()

running = True

while running:
    command = sub.recv_json()

    if 'q' in command:
        controller.stop()
        running = False
    else:
        controller.move(speed = command[0], angSpeed = command[1], duration = command[2])

controller.destroy_node()
rclpy.shutdown()
print("Subscriber has been shut down")
    