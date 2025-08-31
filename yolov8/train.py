from ultralytics import YOLO

model = YOLO("yolov8n-seg.yaml")

model.train(data='/home/right/RIGHT-Infer/datasets/baskettarget_final/data.yaml', epochs=50, batch=16, imgsz=640)