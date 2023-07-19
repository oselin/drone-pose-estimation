import xml.etree.ElementTree as ET
import os, sys

script_directory = os.path.dirname(os.path.abspath(__file__))

def new_drone(index):
    # Create the 'model' element
    model = ET.Element("model")
    model.set("name", f"drone{index}")
    model.text = "\n\t\t"

    # Create the 'pose' subelement
    pose = ET.SubElement(model, "pose")
    pose.text = f"{index} 0 0 0 0 0"
    pose.tail = "\n\t\t"

    # Create the 'include' element
    include = ET.SubElement(model, "include")
    include.text = "\n\t\t\t"
    include.tail = "\n\t"

    # Create the 'uri' element inside 'include' and set its value
    uri = ET.SubElement(include, "uri")
    uri.text = f"model://drone{index}"
    uri.tail = "\n\t\t"
    model.tail = "\n"

    return model

def generate_world(drones_number):

    # Open the XML file
    tree = ET.parse(os.path.join(script_directory,"../worlds/multi_drone_empty.world"))
    root = tree.getroot()

    # Add the right number of drones
    for i in range(1, drones_number+1):
        print(f"Adding drone to world {i}/{drones_number}")
        root.find('.//world').append(new_drone(i))

    # Save the modified XML 
    print("Saving world to file")
    tree.write(os.path.join(script_directory,"../worlds/multi_drone.world"), encoding="utf-8", xml_declaration=True)

if __name__ == "__main__":
    
    if (len(sys.argv) == 2):
        generate_world(int(sys.argv[1]))