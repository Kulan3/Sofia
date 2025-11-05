#  _____       _  _        ___ __   __
# |_   _| ___ | || | ___  / __|\ \ / /
#   | |  / -_)| || |/ _ \| (__  \   /
#   |_|  \___||_||_|\___/ \___|  \_/

import cv2
from ultralytics import YOLO
from djitellopy import Tello

# Initialize Tello
tello = Tello()
tello.connect()
print("Battery:", tello.get_battery())
tello.streamon()

# Load YOLOv8 model
model = YOLO('bp.pt', task='detect')

# Get the latest frame reader
frame_read = tello.get_frame_read(with_queue=False, max_queue_len=0)

try:
    while True:
        frame = frame_read.frame
        if frame is None:
            print("No frame received")
            continue

        # Run detection
        results = model(frame)

        # Draw annotations (Ultralytics returns BGR suitable for cv2.imshow)
        annotated = results[0].plot()

        # Show window
        cv2.imshow("YOLOv8 Tello Drone Tracking", annotated)

        # Press 'x' to exit
        if cv2.waitKey(1) & 0xFF == ord('x'):
            break

finally:
    # Clean up
    tello.end()
    cv2.destroyAllWindows()
