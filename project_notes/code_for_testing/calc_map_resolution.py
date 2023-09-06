# this is used to calculate the resolution of an image to use in the .yaml file
# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/calc_map_resolution.py
from PIL import Image

# Open the image file
img_backyard = Image.open("/home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/Screenshot_of_backyard_120x211.png")

# Convert the image to grayscale
img_backyard_gray = img_backyard.convert('L')

# Save the grayscale image
img_backyard_gray.save('/home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/Screenshot_of_backyard_120x211_gray.png')

# Get the image dimensions
width, height = img_backyard_gray.size

# Convert feet to meters
backyard_width_meters = 211 * 0.3048 
backyard_height_meters = 120 * 0.3048  # approximately 36 meters

# Calculate resolution
resolution_x = backyard_width_meters / width
resolution_y = backyard_height_meters / height

# The resolution should be the same in both directions, so take the average
resolution_avg = (resolution_x + resolution_y) / 2

print(resolution_avg)

