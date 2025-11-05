import cv2
from ultralytics import YOLO
from djitellopy import Tello

# Initialize Tello
tello = Tello()
tello.connect()
print("Battery:", tello.get_battery())
tello.streamon()


model = YOLO('bp.pt', task='detect')


frame_read = tello.get_frame_read(with_queue=False, max_queue_len=0)

try:
    while True:
        frame = frame_read.frame
        if frame is None:
            print("No frame received")
            continue

        results = model(frame)

        annotated = results[0].plot()

        cv2.imshow("YOLOv8 Tello Drone Tracking", annotated)

        if cv2.waitKey(1) & 0xFF == ord('x'):
            break

finally:
    tello.end()
    cv2.destroyAllWindows()
