#!/usr/bin/env python3

import re, sys, os

# Define the file name
filename = os.path.expanduser('~/ardupilot/Tools/autotest/pysim/vehicleinfo.py')

# Define the new content template
CONTENT_TEMPLATE = lambda idx : '\t\t\t"gazebo-drone%i": {\n\t\t\t\t"waf_target": "bin/arducopter",\n\t\t\t\t"default_params_filename": \t["default_params/copter.parm",\n\t\t\t\t\t\t\t\t\t\t\t "default_params/gazebo-drone%i.parm"],\n\t\t\t},\n' % (idx,idx)

def remove_old_content(filename, pattern_start, pattern_stop):
    # Read from file
    with open(filename, 'r') as file: content = file.read()

    match = re.search(pattern_start, content)
    while (match is not None):
        # Look for start and stop points
        start_point = match.start()
        end_point   = re.search(pattern_stop, content[start_point:]).end() + start_point

        # Shrink the content
        content = content[:start_point] + content[end_point:]

        # Look for new content to be removed
        match = re.search(pattern_start, content)

    # Write on file
    with open(filename, 'w') as file: file.write(content)


def add_new_content(filename, pattern_start, new_content):
    # Read from file
    with open(filename, 'r') as file: content = file.read()

    match = re.search(pattern_start, content)

    if (match):
        insert_index = match.end()

    else:
        raise ValueError(f"The pattern '{pattern_start}' was not found in the file.")

    # Insert the new line at the found position
    content = content[:insert_index] + new_content + content[insert_index:]

    # Write the modified content back to the file
    with open(filename, 'w') as file: file.write(content)

if __name__ == "__main__":

    if (len(sys.argv) == 2):
        # Remove old content
        remove_old_content(filename=filename, pattern_start='"gazebo-drone', pattern_stop='},')

        n_drones = int(sys.argv[1])
        pattern = r"\# SIM\n"

        for i in range(1, n_drones + 1):
            add_new_content(filename=filename, pattern_start=pattern, new_content=CONTENT_TEMPLATE(i))
    else:
        raise ValueError("Number of drones not provided!")
        
