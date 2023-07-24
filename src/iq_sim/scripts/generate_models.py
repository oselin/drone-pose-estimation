#!/usr/bin/env python3

import shutil
import xml.etree.ElementTree as ET
import os, sys


directory   = os.path.dirname(os.path.abspath(__file__)) + '/../models/'
folder_base = "drone"
template    = "drone_template"


def remove_old_models(folder_name=None):

    existing_folders = os.listdir(directory)

    if (folder_name is not None):
        for folder in existing_folders:
            if (folder_name in folder and not folder == template):
                shutil.rmtree(directory + folder)


def modify_config_file(dir,index):
    # Open the XML file
    tree = ET.parse(dir + "model.config")
    root = tree.getroot()
    
    # Change the name tag accordingly
    root.find(".//name").text = f"drone{index}"
       
    # Save the modified XML 
    tree.write(os.path.join(dir + "model.config"), encoding="utf-8", xml_declaration=True)


def modify_sdf_file(dir, index):
    # Open the XML file
    tree = ET.parse(dir + "model.sdf")
    root = tree.getroot()

    # Change IN and OUT ports
    for port_in  in root.findall('.//fdm_port_in' ): port_in.text  = str(9002 + index*10)
    for port_out in root.findall('.//fdm_port_out'): port_out.text = str(9003 + index*10)
       
    # Save the modified XML 
    tree.write(os.path.join(dir + "model.sdf"), encoding="utf-8", xml_declaration=True)


def generate_models(n_drones):
    
    # Remove possible old models
    remove_old_models(folder_name=folder_base)

    # Generate new models
    for i in range(1, n_drones + 1):
        try :
            model = folder_base + f"{i}/"
            shutil.copytree(directory + template, directory + model)
            
            # Tailor the new model
            modify_config_file(directory + model, i)
            modify_sdf_file(directory + model, i-1)

        except Exception as e:
            print('Directory not copied.')
            print(e)



if __name__ == "__main__":
    
    if (len(sys.argv) == 2):
        generate_models(int(sys.argv[1]))