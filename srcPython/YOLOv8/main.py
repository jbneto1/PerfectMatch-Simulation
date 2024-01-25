import socket as pysocket
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
ip = "192.168.1.183" # WINDOWS IP HOME
# ip = "193.137.108.42" # WINDOWS IP CEDRI
ip_wsl2 = "172.20.35.129"
port_simtwo = "9899"

#NETWORK DEFINES FOR READY MSG
port_syncMsg = 9890
port_yoloMsg = 9010

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

# Create UDP socket for sending YOLO data to the C++ server
yolo_sock = pysocket.socket(pysocket.AF_INET, pysocket.SOCK_DGRAM)


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
    
# Function to send YOLO data to the C++ server
def send_yolo_data(yolo_data):
    try:
        yolo_sock.sendto(yolo_data.encode(), (ip_wsl2, port_yoloMsg))  # Sending to the C++ application
    except Exception as e:
        print(f"Error sending YOLO data: {e}")
    
try:
    with open(log_file_name, 'a') as log_file:
        send_ready_message()
        while running:
            try:
                message = socket.recv()
                decoded = np.frombuffer(message, np.uint8)
                decoded = decoded.reshape((480, 640, 4))
                decoded = cv2.flip(decoded, 0)
                decoded = np.delete(decoded, 3, 2)

                # Run YOLO inference
                results = model([decoded], stream=True, classes=0)
                
                # Get the current timestamp
                now = datetime.datetime.now()
                timestamp = int(now.timestamp() * 1000)  # Millisecond precision
                
                for result in results:
                    annotated_frame = result.plot()
                    boxes = result.boxes
                    if (len(boxes.cls) != 0):
                        log_str = ''
                        for (iter,box) in enumerate(boxes.data):
                            x1, y1, x2, y2, conf, cls = box[:6].tolist()
                            cls = int(cls)
                            log_str = log_str +  f"b{iter}:{cls},{conf},{x1},{y1},{x2},{y2},"

                        log_str = log_str[:-1]
                        # log_yolo_data(log_str, log_file)
                        if log_str:
                            send_yolo_data(log_str)
                    
                cv2.imshow("YOLOv8.1 Videostream", annotated_frame)
                
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
    yolo_sock.close()
    context.term()
    cv2.destroyAllWindows()
    print("Resources released. Program terminated.")
