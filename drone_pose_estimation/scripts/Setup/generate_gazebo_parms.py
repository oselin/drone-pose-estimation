#!/usr/bin/env python3

import os, sys

folder_path = os.path.expanduser("~/ardupilot/Tools/autotest/default_params/")

def remove_parm(filename=None):
    parm_files = os.listdir(folder_path)
    
    if (filename is not None):

        print("Removing old parameter files")
        for f in parm_files:
            if (filename in f): os.remove(folder_path + f)

def main(n_drones):

    # Define the file name base
    filename_base = "gazebo-drone"

    # Be sure to have the exact parm files
    remove_parm(filename = filename_base)

    for i in range(1, n_drones+1):
        # Define the content to be written to .parm file
        out = ""
        out += "# Iris is X frame"   + "\n"   
        out += "FRAME_CLASS 1"       + "\n"
        out += "FRAME_TYPE  1"       + "\n"
        out += "# IRLOCK FEATURE"    + "\n"   
        out += "RC8_OPTION 39"       + "\n"
        out += "PLND_ENABLED    1"   + "\n"
        out += "PLND_TYPE       3"   + "\n"
        out += "# SONAR FOR IRLOCK"  + "\n"
        out += "SIM_SONAR_SCALE 10"  + "\n"
        out += "RNGFND1_TYPE 1"      + "\n"
        out += "RNGFND1_SCALING 10"  + "\n"
        out += "RNGFND1_PIN 0"       + "\n"   
        out += "RNGFND1_MAX_CM 5000" + "\n"   
        out += f"SYSID_THISMAV {i}"  + "\n"   

        # Define file name
        file_name = filename_base + f"{i}.parm"

        # Write the file
        with open(folder_path + file_name, "w") as file: 
            print(f"Writing new parameter file [{i}/{n_drones}]")
            file.write(out)


if __name__ == "__main__":

    if (len(sys.argv) ==  2):
        main(int(sys.argv[1]))