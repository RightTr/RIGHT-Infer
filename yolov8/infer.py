from ultralytics import YOLO
import cv2
import os

def load_model(weight_path):
    model = YOLO(weight_path)
    return model

def process_video(video_path, model):
    cap = cv2.VideoCapture(video_path)

    if not cap.isOpened():
        print("Open video error!")
        return
    
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('./workspace/detect_video.mp4', fourcc, 30.0, (width, height))

    while(1):
        ret, frame = cap.read()

        if not ret:
            print("End of video stream!")
            break

        result = model(frame)

        frame_boxes = result[0].plot()

        out.write(frame_boxes)

        cv2.imshow("Detected Video", frame_boxes)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    out.release()

if __name__ == "__main__":

    weight_path = '/home/right/Infer/workspace/best_seg.pt'  
    video_path = '/home/right/Infer/workspace/video.mp4'  

    model = load_model(weight_path)

    process_video(video_path, model)

