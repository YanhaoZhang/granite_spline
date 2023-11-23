import cv2
import numpy as np
import os

from os import listdir
from os.path import isfile, join

# Given parameters
intrinsics = {
    "fx": 897.4764841120259,
    "fy": 900.131314815222,
    "cx": 516.5816593053271,
    "cy": 377.03405845062315
}

resolution = [751, 1026]
distortion_coeff = np.array([-0.11106452180588008, 0.17858247476424663, -0.0006877836355554949, -0.00013122812418964706])

# Convert intrinsics to matrix form
camera_matrix = np.array([
    [intrinsics["fx"], 0, intrinsics["cx"]],
    [0, intrinsics["fy"], intrinsics["cy"]],
    [0, 0, 1]
])


raw_dir = "/media/yanhao/8tb1/00_data_TII/island_localization/frames/mav0/cam0/data_raw/"
save_dir = "/media/yanhao/8tb1/00_data_TII/island_localization/frames/mav0/cam0/data/"

# onlyfiles =
# png_files = [f for f in os.listdir(folder_path) if f.endswith('.png')]
onlyfiles = [f for f in os.listdir(raw_dir) if f.endswith('.png')]
onlyfiles = sorted(onlyfiles)


# Load the image
for name in onlyfiles:
    image_path = raw_dir + name
    image = cv2.imread(image_path)

    # Undistort the image
    undistorted_image = cv2.undistort(image, camera_matrix, distortion_coeff)

    # Save the undistorted image
    output_path = save_dir + name
    cv2.imwrite(output_path, undistorted_image)

    a=1

# Compute the new intrinsic matrix (this will be same as the original matrix for pinhole cameras)
new_camera_matrix = camera_matrix

print("Original Camera Matrix:")
print(camera_matrix)
print("\nNew Camera Matrix (after undistortion):")
print(new_camera_matrix)
