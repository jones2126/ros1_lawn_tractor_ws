#!/usr/bin/env python

'''

$ python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/test_range_drop.py
'''
print("Running test_range_drop.py.py")

import pandas as pd

# Sample DataFrame
data = {'X': range(830), 'Y': range(830)}
df = pd.DataFrame(data)

print("Original DataFrame:")
print(df.iloc[825:830])

# Indices to delete
indices_to_delete = [826, 827, 828]

# Delete rows in reverse order to avoid shifting issues
for index in sorted(indices_to_delete, reverse=True):
    df.drop(index, inplace=True)

# Verify that the rows have been deleted
print("DataFrame after deletion:")
print(df.iloc[825:830])

# New row data to be inserted at position 826
new_row = {'X': 1000, 'Y': 1000}

# Insert the new row at position 826
# Split the DataFrame into two parts and insert the new row in between
df_part1 = df.iloc[:826]
df_part2 = df.iloc[826:]

# Reindex the second part
df_part2.index += 1

# Insert the new row
df = pd.concat([df_part1, pd.DataFrame([new_row], index=[826]), df_part2])

# Verify the insertion
print("DataFrame after inserting new row:")
print(df.iloc[825:830])

# the second approach
print("the second approach")

# Sample DataFrame
data = {'X': range(830), 'Y': range(830)}
df = pd.DataFrame(data)

print("Original DataFrame:")
print(df.iloc[825:830])

# Indices to delete
indices_to_delete = [826, 827, 828]

# Delete rows in reverse order to avoid shifting issues
for index in sorted(indices_to_delete, reverse=True):
    df.drop(index, inplace=True)

# Verify that the rows have been deleted
print("DataFrame after deletion:")
print(df.iloc[825:830])

# New row data to be inserted at position 826
new_row = {'X': 1000, 'Y': 1000}

# Insert the new row at position 826
# Split the DataFrame into two parts and insert the new row in between
df_part1 = df.iloc[:826]
df_part2 = df.iloc[826:]

# Reset the index for df_part2 to ensure proper alignment
df_part2.index = df_part2.index + 1

# Insert the new row
df = pd.concat([df_part1, pd.DataFrame([new_row], index=[826]), df_part2])

# Reset the index of the DataFrame to maintain a continuous index
df = df.reset_index(drop=True)

# Verify the insertion
print("DataFrame after inserting new row:")
print(df.iloc[825:830])

# approach 3
print("approach 3")
import pandas as pd

# Sample DataFrame
data = {'X': range(830), 'Y': range(830)}
df = pd.DataFrame(data)

print("Original DataFrame:")
print(df.iloc[825:835])

# Indices to delete
indices_to_delete = [826, 827, 828]

# Delete rows in reverse order to avoid shifting issues
for index in sorted(indices_to_delete, reverse=True):
    df.drop(index, inplace=True)

# Verify that the rows have been deleted
print("DataFrame after deletion:")
print(df.iloc[825:835])

# # New rows data to be inserted at position 826
# new_rows = pd.DataFrame({
#     'X': [1000, 1001, 1002, 1003],
#     'Y': [1000, 1001, 1002, 1003]
# })

new_rows = pd.DataFrame({
    'X': [1000, 1001],
    'Y': [1000, 1001]
})

# Insert the new rows at position 826
# Split the DataFrame into two parts and insert the new rows in between
df_part1 = df.iloc[:826]
df_part2 = df.iloc[826:]

# Adjust the index for df_part2 to ensure proper alignment without overwriting
df_part2.index = df_part2.index + len(new_rows)

# Concatenate the parts
df = pd.concat([df_part1, new_rows, df_part2])

# Reset the index of the DataFrame to maintain a continuous index
df = df.reset_index(drop=True)

# Verify the insertion
print("DataFrame after inserting new rows:")
print(df.iloc[825:835])
