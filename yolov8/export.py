from ultralytics import YOLO
import onnx

model = YOLO("/home/right/Infer/workspace/best_seg.pt")

success = model.export(format="onnx", dynamic=True, simplify=True)
