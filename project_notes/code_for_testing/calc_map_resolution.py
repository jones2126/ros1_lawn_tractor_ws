# this is used to calculate the resolution of an image to use in the .yaml file
# $ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/calc_map_resolution.py
from PIL import Image
import os

# Full path to the image file
image_name = 'Collins_62_192X198'
image_type = 'png'
image_path = "/home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/"
land_height = 192
land_width = 198

# Set the working directory
os.chdir(image_path)

# Convert the image to grayscale
image_full_path = image_path + image_name + '.' + image_type
land_image = Image.open(image_full_path)
land_image_gray = land_image.convert('L')
grayscale_file_name = image_name + '_gray.png'
land_image_gray.save(grayscale_file_name)

# Calculate Map Resolution (in terms of distance per pixel)
img_width, img_height = land_image.size
land_height_meters = land_height * 0.3048   # North/South
land_width_meters = land_width * 0.3048 	# East/West
resolution_x = land_width_meters / img_width
resolution_y = land_height_meters / img_height
map_resolution_avg = (resolution_x + resolution_y) / 2   

print(map_resolution_avg)

# Create a text file using the image name
text_file_name = image_name + '.txt'

# Write the required information to the text file
with open(text_file_name, 'w') as file:
    file.write(f"Image Name: {image_name}\n")
    file.write(f"Map Resolution Average: {map_resolution_avg}\n")
    file.write(f"Image height and width in pixels: {img_height} x {img_width}\n")
    # file.write(f"avg_lat: {avg_lat}\navg_lon: {avg_lon}\n")

print(f"Information written to {text_file_name}")
