import zmq

context = zmq.Context()

pub = context.socket(zmq.PUB)
pub.bind("tcp://*:5555")
print("starting publisher...")

print("Enter 3 numbers separated by spaces or 'q' to quit")
print("Order of input: linear speed, angular speed, duration")

running = True

while running:
    try:
        command = input("⇨ ").split()
    
        if 'q' in command:
            running = False
        else: 
            for i in range(len(command)):
                command[i] = float(command[i])
    except:
        print("Invalid command format. Try again.")

    else:
        print("Sending command to car")
        pub.send_json(command)
        

print("Quitting")