import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import xml.etree.ElementTree as ET
import re

### --- Step 1: Parse graph edges from .txt --- ###
def parse_graph_edges(file_path):
    edges = []
    with open(file_path, 'r') as file:
        text = file.read()

    pattern = r"From\s+\((\d+),(\d+),(\d+)\)\s+to:\s+((?:\(\d+,\d+,\d+\)\s*)+)"
    matches = re.findall(pattern, text)

    for match in matches:
        src = tuple(map(int, match[:3]))
        destinations = re.findall(r"\((\d+),(\d+),(\d+)\)", match[3])
        for dest in destinations:
            dest_pos = tuple(map(int, dest))
            edges.append((src, dest_pos))
    return edges

### --- Step 2: Parse positions from XML --- ###
def parse_block_positions(xml_path):
    tree = ET.parse(xml_path)
    root = tree.getroot()

    blocks = []
    for block in root.findall(".//blockList/block"):
        pos_str = block.get("position")
        pos = tuple(map(int, pos_str.split(',')))
        blocks.append(pos)
    return blocks

### --- Step 3: Plot it all --- ###
def plot_3d_graph(edges, block_positions):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Adjust points for staggered Z alignment
    graph_nodes = set([pt for edge in edges for pt in edge])
    adjusted_edges = [(apply_z_layer_offset([src])[0], apply_z_layer_offset([dest])[0]) for src, dest in edges]
    adjusted_nodes = apply_z_layer_offset(graph_nodes)
    adjusted_blocks = apply_z_layer_offset(block_positions)

    # Plot edges
    for src, dest in adjusted_edges:
        ax.plot([src[0], dest[0]],
                [src[1], dest[1]],
                [src[2], dest[2]],
                color='skyblue', alpha=0.5, linewidth=1)

    # Plot graph nodes
    gx, gy, gz = zip(*adjusted_nodes)
    ax.scatter(gx, gy, gz, c='crimson', s=50, marker='o', label='Graph Nodes')

    # Plot XML blocks
    if adjusted_blocks:
        bx, by, bz = zip(*adjusted_blocks)
        ax.scatter(bx, by, bz, c='limegreen', s=70, marker='s', edgecolors='black', label='XML Blocks')

    # Combine all for scaling
    all_points = adjusted_nodes + adjusted_blocks
    xs, ys, zs = zip(*all_points)

    # Axis aesthetics
    ax.set_xlabel('X', fontsize=12)
    ax.set_ylabel('Y', fontsize=12)
    ax.set_zlabel('Z', fontsize=12)
    ax.set_title(' 3D Graph Visualization with XML Blocks (Z-layer aligned)', fontsize=14, weight='bold')

    # Limits
    margin = 2
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_zlim(min(zs) - margin, max(zs) + margin)

    ax.grid(True)
    ax.view_init(elev=30, azim=45)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.show()


def apply_z_layer_offset(points):
    """
    Applies +0.5 X-offset to all positions where Z is even.
    """
    adjusted = []
    for x, y, z in points:
        if z % 2 == 0:
            x -= 0.5
            y -= 0.5
        adjusted.append((x, y, z))
    return adjusted


### --- Main --- ###
if __name__ == "__main__":
    graph_file = "applicationsBin/Graphtest/graph_edges.txt"
    xml_file = "applicationsBin/Graphtest/config1.xml"  # Replace with your actual file path

    edges = parse_graph_edges(graph_file)
    blocks = parse_block_positions(xml_file)
    plot_3d_graph(edges, blocks)
