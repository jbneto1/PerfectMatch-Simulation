import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)

# Binding to localhost on port 9899
socket.bind("tcp://*:9899")

try:
    while True:
        # Sending a simple text message
        socket.send_string("Hello from Publisher")
        print("Message sent")
        time.sleep(1)  # Wait for 1 second

except KeyboardInterrupt:
    pass
finally:
    socket.close()
    context.term()
