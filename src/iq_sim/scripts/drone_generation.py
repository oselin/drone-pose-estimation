import xml.etree.ElementTree as ET
from xml.dom import minidom


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
    tree = ET.parse("../worlds/multi_drone_empty.world")
    root = tree.getroot()

    # Add the right number of drones
    for i in range(drones_number):
        root.append(new_drone(i))

    # Save the modified XML 
    tree.write("../worlds/multi_drone.world", encoding="utf-8", xml_declaration=True)

if __name__ == "__main__":
    number_drones = 2
    generate_world(number_drones)