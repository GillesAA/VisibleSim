import xml.etree.ElementTree as ET

def parse_position(position):
    """Parse the position string into a tuple of integers."""
    return tuple(map(int, position.split(',')))

def filter_blocks(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    block_list = root.find('blockList')
    if block_list is None:
        print("No blockList found in the XML.")
        return
    
    blocks = block_list.findall('block')
    blocks_to_remove = []

    for block in blocks:
        position = parse_position(block.get('position'))
        # Only process blocks with z between 0 and 3
        if 0 <= position[2] <= 3:
            # Check for a block with the same x, y and z+1
            successor_exists = any(
                parse_position(b.get('position')) == (position[0], position[1], position[2] + 4)
                for b in blocks
            )
            if not successor_exists:
                blocks_to_remove.append(block)

    # Remove the marked blocks
    for block in blocks_to_remove:
        block_list.remove(block)

    # Save the modified tree to a new file (optional)
    tree.write('Carlita_c.xml')

# Replace 'your_file.xml' with the path to your XML file
filter_blocks('Carlita_cube.xml')
