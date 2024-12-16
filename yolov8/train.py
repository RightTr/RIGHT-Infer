from ultralytics import YOLO

model = YOLO("yolov8n-seg.yaml")

model.train(data='/home/right/Infer/datasets/basket_seg/data.yaml', epochs=50, batch=16, imgsz=640)