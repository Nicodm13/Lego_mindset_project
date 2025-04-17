import cv2
from ultralytics import YOLO

model = YOLO('model1.pt')
results = model('examples/ping-pong-balls.jpg', imgsz=640)
results[0].show()