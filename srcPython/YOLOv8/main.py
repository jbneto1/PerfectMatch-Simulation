import zmq
import numpy as np
import cv2
from ultralytics import YOLO
import torch
import datetime
import time

# YOLO setup
torch.cuda.set_device(0)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('yolov8n.pt').to(device)

# ZMQ setup
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.1.183:9899") #windows's IP
socket.setsockopt_string(zmq.SUBSCRIBE, '')

def log_yolo_data(box_data):
    # Log YOLO data to a file with timestamp
    with open('yolo_detections.txt', 'a') as file:
        file.write(box_data + '\n')
        
# Function to get formatted current datetime similar to C++ code
def current_datetime():
    return datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

# File naming
filename_yolo = f"../../docs/logs/yolo_{current_datetime()}.txt"
        
# Open the file
with open(filename_yolo, 'w') as file:
    try:
        while True:
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
                    # Extract bounding box coordinates and other info
                    x1, y1, x2, y2, conf, cls = box[:6].tolist()
                    cls = int(cls)
                    
                    if cls == 0:  # Assuming 0 is the class ID for 'person'
                        # Format the data to log
                        log_str = f"{x1},{y1},{x2},{y2},{conf},{cls},{timestamp}\n"
                        file.write(log_str)

                        #Debug
                        print(f"BB coordinates: {x1, y1, x2, y2}, Class ID: {cls}, Confidence: {conf:.2f}")
                        label = f'Person {conf:.2f}'
                        cv2.rectangle(decoded, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(decoded, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            cv2.imshow("YOLOv8.1 Videostream", decoded)
            if cv2.waitKey(1) == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        socket.close()
        context.term()
        cv2.destroyAllWindows()
