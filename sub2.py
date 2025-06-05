import zmq
import time
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
    comm = sub.recv_string()

    if comm == "w":
        controller.drive() # default speed = 0.1

    elif comm == "s":
        controller.drive(speed = -0.1)

    elif comm == "a":
        controller.turn() # left at 0.1 rad/s

    elif comm == "d":
        controller.turn(angSpeed = -0.1)    # right

    elif comm == "q":
        # Exit
        controller.stop() 
        running = False

    else:
        # pause if given unexpected commands
        controller.stop()
    
controller.destroy_node()
rclpy.shutdown()
print("Subscriber has been shut down")
    