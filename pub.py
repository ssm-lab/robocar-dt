import zmq
## message sender

context = zmq.Context()

pub = context.socket(zmq.PUB)
pub.bind("tcp://*:5555")
print("starting publisher...")
print("Enter a command to move car (w/s/a/d or q to quit)")
comm = input("⇨ ")
running = True
while running:
    print("Sending command: ", comm)
    pub.send_string(comm)
    if comm!= "q":
        comm = input("⇨ ")
    else:
        running = False
        

print("Quitting")