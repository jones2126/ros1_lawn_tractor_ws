#!/usr/bin/env python3
# $ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/rosbag_utilities/calculateGforce.py
# 
#!/usr/bin/env python3
import pandas as pd
import numpy as np

# File paths
original_file_path = '/home/tractor/wheel_data_right.csv'
output_file_path = '/home/tractor/wheel_data_right_with_gforce.csv'

# Read the original CSV file
df = pd.read_csv(original_file_path)

# Initialize variables
prev_speed = 0
time_step = 0.1  # 10 Hz sampling rate
error_count = 0  # Counter for error conditions
min_g_force = float('inf')  # Initialize to positive infinity
max_g_force = float('-inf')  # Initialize to negative infinity
min_g_force_index = -1
max_g_force_index = -1
g_force_threshold = 3

# Calculate G-force for each record
g_forces = []
for index, row in df.iterrows():
    current_speed = row['speed']
    
    # Calculate acceleration and G-force
    acceleration = (current_speed - prev_speed) / time_step
    g_force = acceleration / 9.81
    g_forces.append(g_force)
    
    # Check if G-force exceeds threshold
    if abs(g_force) > g_force_threshold:
        print(f"Record index: {index}, Previous speed: {prev_speed:.2f}, Current speed: {current_speed:.2f}, G-force: {g_force:.2f}")
        error_count += 1
    else:
        # Update min and max G-force if not an error condition
        if g_force < min_g_force:
            min_g_force = g_force
            min_g_force_index = index
        if g_force > max_g_force:
            max_g_force = g_force
            max_g_force_index = index
    
    # Update previous speed for next iteration
    prev_speed = current_speed

# Add G-force as a new column to the original DataFrame
df['G_Force'] = g_forces

# Write the combined result to a new CSV file
df.to_csv(output_file_path, index=False)

print(f"Processing complete. Results written to {output_file_path}")

# Print the total error count divided by 3
print(f"Total error conditions: {error_count}")
print(f"Error count divided by 3: {error_count / 3:.2f}")

# Print min and max G-force excluding error conditions, along with their indices
print(f"Minimum G-force (excluding error conditions): {min_g_force:.2f} at record index {min_g_force_index}")
print(f"Maximum G-force (excluding error conditions): {max_g_force:.2f} at record index {max_g_force_index}")