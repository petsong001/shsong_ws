import xml.etree.ElementTree as ET
import os

# Path to your SRDF file
file_path = os.path.expanduser('~/shsong_ws/src/ur5e_hande_complete/config/ur5e_hande.srdf')

print(f"Cleaning SRDF file: {file_path}")
tree = ET.parse(file_path)
root = tree.getroot()

# Items to remove
to_remove = []

# Find all groups, group_states, or end_effectors related to the gripper
for child in root:
    # Remove the 'gripper' group definition
    if child.tag == 'group' and child.attrib.get('name') == 'gripper':
        to_remove.append(child)
    # Remove states defined for the gripper
    if child.tag == 'group_state' and child.attrib.get('group') == 'gripper':
        to_remove.append(child)
    # Remove the end_effector definition
    if child.tag == 'end_effector' and child.attrib.get('name') == 'gripper':
        to_remove.append(child)

# Execute removal
for item in to_remove:
    print(f"Removing tag: {item.tag} name={item.attrib.get('name')}")
    root.remove(item)

# Save back to file
tree.write(file_path)
print("SRDF cleaned successfully!")
