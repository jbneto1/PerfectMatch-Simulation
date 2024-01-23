import socket as pysocket  # Import socket module for UDP communication
import zmq
import numpy as np
import cv2
from ultralytics import YOLO
import torch
import datetime
import signal

# Flag to control the main loop
running = True

# NETWORK DEFINES for SIMTWO comm
# ip = "192.168.1.183" # WINDOWS IP HOME
ip = "193.137.108.164" # WINDOWS IP CEDRI
ip_wsl2 = "172.20.35.129"
port_simtwo = "9899"

#NETWORK DEFINES FOR READY MSG
port_syncMsg = 9890

def signal_handler(sig, frame):
    global running
    print('SIGINT received, terminating the program...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

print("YOLOV8.1 script starting...")

# YOLO setup
torch.cuda.set_device(0)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('yolov8n.pt').to(device)

# ZMQ setup
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(f"tcp://{ip}:{port_simtwo}") #windows's IP CeDRI
socket.setsockopt_string(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.RCVTIMEO, 500)  # Set to non-blocking with a timeout of ms
# Function to log YOLO data
def log_yolo_data(box_data, file):
    file.write(box_data + '\n')

# Function to get formatted current datetime
def current_datetime():
    return datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

log_file_name = f"../../docs/logs/yolo_{current_datetime()}.txt"

# Create a named window and set its size
cv2.namedWindow("YOLOv8.1 Videostream", cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow("YOLOv8.1 Videostream", 800, 600)

# Function to send a ready message to the C++ server
def send_ready_message():
    message = b"ready"  # Message to be sent
    print(f"Sending ready message to {ip}:{port_syncMsg}")
    sock = pysocket.socket(pysocket.AF_INET, pysocket.SOCK_DGRAM)
    sock.sendto(message, (ip_wsl2, port_syncMsg))
    sock.close()
    
send_ready_message()

try:
    with open(log_file_name, 'a') as log_file:
        while running:
            try:
                message = socket.recv()
                decoded = np.frombuffer(message, np.uint8)
                decoded = decoded.reshape((480, 640, 4))
                decoded = cv2.flip(decoded, 0)
                decoded = np.delete(decoded, 3, 2)

                frame = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)

                # Run YOLO inference
                results = model([frame])

                # Get the current timestamp
                now = datetime.datetime.now()
                timestamp = int(now.timestamp() * 1000)  # Millisecond precision

                # Process results
                for result in results:
                    boxes = result.boxes

                    for box in boxes.data:  # Iterate through each box
                        x1, y1, x2, y2, conf, cls = box[:6].tolist()
                        cls = int(cls)
                        
                        if cls == 0:  # Assuming 0 is the class ID for 'person'
                            log_str = f"{x1},{y1},{x2},{y2},{conf},{cls},{timestamp}"
                            log_yolo_data(log_str, log_file)
                            
                            print(f"BB coordinates: {x1, y1, x2, y2}, Class ID: {cls}, Confidence: {conf:.2f}")
                            label = f'Person {conf:.2f}'
                            cv2.rectangle(decoded, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(decoded, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                cv2.imshow("YOLOv8.1 Videostream", decoded)
                if cv2.waitKey(1) == ord('q'):
                    break
            except zmq.Again:
                continue
except KeyboardInterrupt:
    print("Keyboard interrupt detected. Shutting down...")
except Exception as e:
    print(f"Caught an exception: {e}")
finally:
    print("Cleaning up resources...")
    socket.close()
    context.term()
    cv2.destroyAllWindows()
    print("Resources released. Program terminated.")
