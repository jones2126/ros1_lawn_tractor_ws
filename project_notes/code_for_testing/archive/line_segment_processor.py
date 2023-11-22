
import pandas as pd
import numpy as np

def calculate_length(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_slope(x1, y1, x2, y2):
    if x2 == x1:
        return np.nan  # Undefined slope for vertical lines
    else:
        return (y2 - y1) / (x2 - x1)

def process_csv(input_file_path, output_file_path):
    # Read the CSV file
    line_segments_df = pd.read_csv(input_file_path)

    # Calculate length and slope
    line_segments_df['Length'] = line_segments_df.apply(lambda row: calculate_length(row['x1'], row['y1'], row['x2'], row['y2']), axis=1)
    line_segments_df['Slope'] = line_segments_df.apply(lambda row: calculate_slope(row['x1'], row['y1'], row['x2'], row['y2']), axis=1)

    # Save the updated DataFrame to a new CSV file
    line_segments_df.to_csv(output_file_path, index=False)

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        print("Usage: python <script_name>.py <input_csv_file> <output_csv_file>")
    else:
        input_file_path = sys.argv[1]
        output_file_path = sys.argv[2]
        process_csv(input_file_path, output_file_path)
