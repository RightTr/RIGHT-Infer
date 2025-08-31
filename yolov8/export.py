from ultralytics import YOLO
import onnx

model = YOLO("/home/right/RIGHT-Infer/workspace/BasketTarget_final/best.pt")

success = model.export(format="onnx", dynamic=True, simplify=True)
