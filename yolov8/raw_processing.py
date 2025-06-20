import cv2
import numpy as np
import glob
import os

input_folder = '/home/right/RIGHT-Infer/datasets/basket_ir_2nd'     
output_folder = '/home/right/RIGHT-Infer/datasets/basket_ir_2nd_enhanced'  

image_paths = glob.glob(os.path.join(input_folder, '*.png'))

for path in image_paths:
    gray_image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    rgb_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    enhanced = cv2.detailEnhance(rgb_image, sigma_s=15, sigma_r=0.15) 
    filename = os.path.basename(path)
    save_path = os.path.join(output_folder, filename)
    cv2.imwrite(save_path, enhanced)
    print(f"Converted and saved: {save_path}")

print("All images converted.")
