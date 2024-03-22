import cv2
from ultralytics import YOLO
import torch
import signal

# Flag to control the main loop
running = True

def signal_handler(sig, frame):
    global running
    print('SIGINT received, terminating the program...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

print("YOLOv8.1 script starting...")

# YOLO setup
torch.cuda.set_device(0)
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = YOLO('yolov8n.pt').to(device)

# URL of the video stream (adjust the IP address and port as necessary)
stream_url = "http://193.137.108.168:8000"

# Attempt to open the video stream
cap = cv2.VideoCapture(stream_url)

# Check if the video stream was opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Create a named window for displaying the video
cv2.namedWindow("YOLOv8.1 Videostream", cv2.WINDOW_AUTOSIZE)
cv2.resizeWindow("YOLOv8.1 Videostream", 800, 600)

try:
    while running:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame.")
            break

        # Run YOLO inference with summary info
        results = model([frame], stream=True, classes=0)
        
        for result in results:
            annotated_frame = result.plot()

        annotated_frame = cv2.resize(annotated_frame, (640,480))
        # Display the resulting frame
        cv2.imshow("YOLOv8.1 Videostream", annotated_frame)

        if cv2.waitKey(1) == ord('q'):
            break
except KeyboardInterrupt:
    print("Keyboard interrupt detected. Shutting down...")
except Exception as e:
    print(f"Caught an exception: {e}")
finally:
    print("Cleaning up resources...")
    cap.release()
    cv2.destroyAllWindows()
    print("Resources released. Program terminated.")
