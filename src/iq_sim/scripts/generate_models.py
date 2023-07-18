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


## TODO
# def modify_config_file(index):
#     # Open the XML file
#     tree = ET.parse(os.path.join(script_directory,"../worlds/multi_drone_empty.world"))
#     root = tree.getroot()

#     # Add the right number of drones
#     for i in range(drones_number):
#         root.append(new_drone(i))

#     # Save the modified XML 
#     tree.write(os.path.join(script_directory,"../worlds/multi_drone.world"), encoding="utf-8", xml_declaration=True)

## TODO
# def modify_sdf_file(index):
#     # Open the XML file
#     tree = ET.parse(os.path.join(script_directory,"../worlds/multi_drone_empty.world"))
#     root = tree.getroot()

#     # Add the right number of drones
#     for i in range(drones_number):
#         root.append(new_drone(i))

#     # Save the modified XML 
#     tree.write(os.path.join(script_directory,"../worlds/multi_drone.world"), encoding="utf-8", xml_declaration=True)


def generate_models(n_drones):
    
    # Remove possible old models
    remove_old_models(folder_name=folder_base)

    # Generate new models
    for i in range(1, n_drones + 1):
        try :
            model = folder_base + f"{i}"
            shutil.copytree(directory + template, directory + model)

        except Exception as e:
            print('Directory not copied.')
            print(e)



if __name__ == "__main__":
    
    if (len(sys.argv) == 2):
        generate_models(int(sys.argv[1]))