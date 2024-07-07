#!/usr/bin/env python

'''
Script that looks for search terms in the ROS log folder '/home/tractor/.ros/log/latest.

$ python3 ~/ros1_lawn_tractor_ws/project_notes/code_for_testing/utilities/search_log_files.py
search_terms = ['error', 'warning', 'exception', 'fail', 'critical', 'ROS2portXfer', 'Failed', 'USB2TTL']
'''
import os
import re
from collections import defaultdict

def search_logs(directory, search_terms, whitelist):
    results = {term: defaultdict(list) for term in search_terms}
    file_count = 0
    
    for filename in os.listdir(directory):
        if filename.endswith('.log'):
            file_count += 1
            filepath = os.path.join(directory, filename)
            print(f"Processing file {file_count}: {filename}")
            
            with open(filepath, 'r') as file:
                for line_number, line in enumerate(file, 1):
                    for term in search_terms:
                        if re.search(r'\b' + re.escape(term) + r'\b', line, re.IGNORECASE):
                            # Check if any whitelist term is in the line
                            if not any(wl_term in line for wl_term in whitelist):
                                results[term][filename].append((line_number, line.strip()))
    
    print(f"\nTotal log files processed: {file_count}")
    return results

# Directory containing log files
log_directory = '/home/tractor/.ros/log/latest'

# Terms to search for
#search_terms = ['error', 'warning', 'exception', 'fail', 'critical', 'SerialException', 'Failed', 'USB2TTL']
#search_terms = ['SerialException', 'Failed', 'USB2TTL', 'Starting_ROS2portXfer']
search_terms = ['I2C','warning']

# Whitelist terms
whitelist = ['off_path_error', 'JS ERROR']

print("Starting log file search...")
search_results = search_logs(log_directory, search_terms, whitelist)

print("\nSearch Results:")
for term, files in search_results.items():
    total_occurrences = sum(len(occurrences) for occurrences in files.values())
    print(f"\n'{term}' found {total_occurrences} times in {len(files)} files:")
    
    for filename, occurrences in files.items():
        print(f"  File: {filename} ({len(occurrences)} occurrences)")
        for i, (line_number, line) in enumerate(occurrences[:3], 1):
            print(f"    {i}. Line {line_number}: {line}")
        if len(occurrences) > 3:
            print(f"    ... and {len(occurrences) - 3} more occurrences")