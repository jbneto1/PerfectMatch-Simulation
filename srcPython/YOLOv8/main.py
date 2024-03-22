import socket as pysocket
import zmq
import numpy as np
import cv2
from ultralytics import YOLO
import torch
import datetime
import signal
import time

# Flag to control the main loop
running = True

# NETWORK DEFINES for SIMTWO comm
ip = "192.168.1.79" # WINDOWS IP HOME
# ip = "193.137.108.252" # WINDOWS IP CEDRI
port_simtwo = "9899"

#NETWORK DEFINES FOR READY MSG (WSL2 CODES)
ip_wsl2 = "172.20.35.129"
port_syncMsg = 9890
port_yoloMsg = 9010
port_syncAck = 9009


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
socket.connect(f"tcp://{ip}:{port_simtwo}")
socket.setsockopt_string(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.RCVTIMEO, 500)  # Set to non-blocking with a timeout of ms

# Create UDP socket for sending YOLO data to the C++ server
yolo_sock = pysocket.socket(pysocket.AF_INET, pysocket.SOCK_DGRAM)

# Create a named window and set its size
cv2.namedWindow("YOLOv8.1 Videostream", cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow("YOLOv8.1 Videostream", 800, 600)

def send_ready_message():
    message = b"ready"
    print(f"Sending ready message to {ip_wsl2}:{port_syncMsg}")
    send_sock = pysocket.socket(pysocket.AF_INET, pysocket.SOCK_DGRAM)
    recv_sock = pysocket.socket(pysocket.AF_INET, pysocket.SOCK_DGRAM)
    recv_sock.bind(('', port_syncAck))
    recv_sock.settimeout(1.5)

    try:
        send_sock.sendto(message, (ip_wsl2, port_syncMsg))
        try:
            ack, _ = recv_sock.recvfrom(1024)
            if ack == b"acknowledged":
                print("Acknowledgment received. Starting logging.")
                return True
            else:
                print("Unexpected message received:", ack)
        except pysocket.timeout:
            print("Timeout waiting for acknowledgment.")
    except Exception as e:
        print(f"Error sending ready message: {e}")
    finally:
        send_sock.close()
        recv_sock.close()
    return False
    
# Function to send YOLO data to the C++ server
def send_yolo_data(yolo_data):
    try:
        yolo_sock.sendto(yolo_data.encode(), (ip_wsl2, port_yoloMsg))  # Sending to the C++ application
    except Exception as e:
        print(f"Error sending YOLO data: {e}")
    
try:
    message_received = False
    while(message_received != True and running == True):
        message_received = send_ready_message()
        if (message_received == True):
            break
        else:    
            print("Retrying to send ready message...")

    while running:
        try:
            message = socket.recv()
            decoded = np.frombuffer(message, np.uint8)
            decoded = decoded.reshape((480, 640, 4))
            decoded = cv2.flip(decoded, 0)
            decoded = np.delete(decoded, 3, 2)
            # print(decoded.shape)
            # decoded = cv2.cvtColor(decoded, cv2.COLOR_RGB2GRAY)
            # decoded = cv2.cvtColor(decoded, cv2.COLOR_GRAY2RGB)
            # print(decoded.shape)

            # Run YOLO inference without summary info
            # results = model([decoded], stream=True, classes=0, verbose=False)
            
            # Run YOLO inference with summary info
            results = model([decoded], stream=True, classes=0)
            
            # Get the current timestamp
            now = datetime.datetime.now()
            timestamp = int(now.timestamp() * 1000)  # Millisecond precision
            
            for result in results:
                annotated_frame = result.plot()
                boxes = result.boxes.xywh  # Assuming this is a tensor of shape [N, 4] where N is the number of boxes
                classes = result.boxes.cls  # Assuming this is a tensor of shape [N,]
                confidences = result.boxes.conf  # Assuming this is a tensor of shape [N,]

                if (len(classes) != 0):
                    log_str = 'N,' + str(len(classes)) + ','
                    for i in range(len(classes)):
                        x1, y1, w, h = boxes[i].tolist()  # Correctly indexing the i-th box
                        conf = confidences[i].item()  # Correctly indexing the i-th confidence, converting to Python scalar
                        cls = int(classes[i].item())  # Correctly indexing the i-th class, converting to Python scalar
                        
                        log_str += f"b{cls},{conf:.2f},{x1},{y1},{w},{h},"

                    log_str = log_str[:-1]  # Removing the last comma
                    if log_str:
                        # print(f"Size in bytes: {len(log_str.encode('utf-8'))}")
                        send_yolo_data(log_str)
                else:
                    log_str = 'NoDetections'
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
