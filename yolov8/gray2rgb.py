import cv2
import numpy as np
import glob
import os

input_folder = '/home/right/RIGHT-Infer/datasets/basket_ir'     
output_folder = '/home/right/RIGHT-Infer/datasets/basket_ir_rgb'  

image_paths = glob.glob(os.path.join(input_folder, '*.png'))

for path in image_paths:
    gray_image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    rgb_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2RGB)
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, filename)
    cv2.imwrite(save_path, rgb_image)
    print(f"Converted and saved: {save_path}")

print("All images converted.")
