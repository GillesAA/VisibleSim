import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import xml.etree.ElementTree as ET
import numpy as np
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

### --- Step 3: Parse Path from .txt --- ###
def parse_path(file_path):
    with open(file_path, 'r') as file:
        text = file.read()

    # Extract the path line (customize this if you have other text too)
    path_match = re.search(r"Path:\s*((?:\(\d+,\d+,\d+\)\s*)+)", text)
    if not path_match:
        return []

    path_str = path_match.group(1)
    path_coords = re.findall(r"\((\d+),(\d+),(\d+)\)", path_str)
    return [tuple(map(int, coord)) for coord in path_coords]


### --- Step 4: Plot it all --- ###
def plot_3d_graph(edges, block_positions, path=None):
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    graph_nodes = set([pt for edge in edges for pt in edge])
    adjusted_edges = [(apply_z_layer_offset([src])[0], apply_z_layer_offset([dest])[0]) for src, dest in edges]
    adjusted_nodes = apply_z_layer_offset(graph_nodes)
    adjusted_blocks = apply_z_layer_offset(block_positions)

    # Plot edges using Line3DCollection (much faster)
    edge_lines = [([src[0], dest[0]], [src[1], dest[1]], [src[2], dest[2]]) for src, dest in adjusted_edges]
    line_segments = Line3DCollection([list(zip(x, y, z)) for x, y, z in edge_lines],
                                     colors='skyblue', linewidths=1, alpha=0.5)
    ax.add_collection3d(line_segments)

    # Plot nodes
    if adjusted_nodes:
        gx, gy, gz = zip(*adjusted_nodes)
        ax.scatter(gx, gy, gz, c='crimson', s=30, marker='o', label='Graph Nodes')

    # Plot XML blocks
    if adjusted_blocks:
        bx, by, bz = zip(*adjusted_blocks)
        ax.scatter(bx, by, bz, c='limegreen', s=60, marker='s', edgecolors='black', label='XML Blocks')

    # Highlight path if provided
    if path:
        adjusted_path = apply_z_layer_offset(path)
        px, py, pz = zip(*adjusted_path)
        ax.plot(px, py, pz, color='gold', linewidth=4, label='Highlighted Path')
        ax.scatter(px, py, pz, color='gold', s=60, edgecolors='black', zorder=5, antialiased=False)

    # Scaling
    all_points = adjusted_nodes + adjusted_blocks
    xs, ys, zs = zip(*all_points)
    margin = 2
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - margin, max(ys) + margin)
    ax.set_zlim(min(zs) - margin, max(zs) + margin)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Graph Visualization', fontsize=14, weight='bold')
    ax.grid(True)
    ax.view_init(elev=30, azim=45)
    ax.legend(loc='upper right')

    plt.tight_layout()
    plt.show()



def apply_z_layer_offset(points):
    # Ensure it's a list of tuples (not a set)
    if isinstance(points, set):
        points = list(points)

    points = np.array(points)
    if points.ndim != 2 or points.shape[1] != 3:
        return []  # Fail-safe in case of bad input

    even_z_mask = (points[:, 2] % 2 == 0)
    offset = np.zeros_like(points, dtype=float)
    offset[even_z_mask, 0] = -0.5
    offset[even_z_mask, 1] = -0.5
    return (points + offset).tolist()



### --- Main --- ###
if __name__ == "__main__":
    graph_file = "applicationsBin/Graphtest/graph_edges.txt"
    xml_file = "applicationsBin/Graphtest/config.xml"  # Replace with your actual file path

    edges = parse_graph_edges(graph_file)
    blocks = parse_block_positions(xml_file)
    path = parse_path(graph_file)
    plot_3d_graph(edges, blocks, path)
