# This program is the subscriber receiving the camera data

import zmq
context = zmq.Context()
sub = context.socket(zmq.SUB)
sub.connect("tcp://192.168.149.1:5555")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

print("Starting subscriber")
while True:
    image = sub.recv()
    print(image)