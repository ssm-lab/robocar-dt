from Publisher import Publisher

__author__ = "Adwita Kashyap"
__credits__ = "Istvan David"

"""
User publisher: 
- Runs on the user's device
- Sends car control commands to CarSubscriber.py
"""

class UserPublisher(Publisher):

    def __init__(self):
        super().__init__("5558")

    def publish(self):
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
                self._publisher.send_json(command)
        
if __name__ == '__main__':
    userPublisher = UserPublisher()
    userPublisher.publish()