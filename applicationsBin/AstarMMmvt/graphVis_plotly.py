import plotly.graph_objects as go
import xml.etree.ElementTree as ET
import re
import numpy as np

### --- Step 1: Parse graph edges from .txt --- ###
# def parse_graph_edges(file_path):
#     edges = []
#     with open(file_path, 'r') as file:
#         text = file.read()

#     pattern = r"From\s+\((\d+),(\d+),(\d+)\)\s+to:\s+((?:\(\d+,\d+,\d+\)\s*)+)"
#     matches = re.findall(pattern, text)

#     for match in matches:
#         src = tuple(map(int, match[:3]))
#         destinations = re.findall(r"\((\d+),(\d+),(\d+)\)", match[3])
#         for dest in destinations:
#             dest_pos = tuple(map(int, dest))
#             edges.append((src, dest_pos))
#     return edges
def parse_graph_edges(file_path):
    edges = set()
    edge_counts = {}

    with open(file_path, 'r') as file:
        text = file.read()

    pattern = r"From\s+\((\d+),(\d+),(\d+)\)\s+to:\s+((?:\(\d+,\d+,\d+\)\s*)+)"
    matches = re.findall(pattern, text)

    for match in matches:
        src = tuple(map(int, match[:3]))
        destinations = re.findall(r"\((\d+),(\d+),(\d+)\)", match[3])
        for dest in destinations:
            dest_pos = tuple(map(int, dest))
            edge = (src, dest_pos)
            edges.add(edge)
            edge_counts[edge] = edge_counts.get(edge, 0) + 1

    # Filter only bidirectional edges
    bidirectional_edges = [
        (a, b) for (a, b) in edges if (b, a) in edges
    ]

    return bidirectional_edges


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

### --- Step 3: Parse Path --- ###
def parse_path(file_path):
    with open(file_path, 'r') as file:
        text = file.read()

    path_match = re.search(r"Path:\s*((?:\(\d+,\d+,\d+\)\s*)+)", text)
    if not path_match:
        return []

    path_str = path_match.group(1)
    path_coords = re.findall(r"\((\d+),(\d+),(\d+)\)", path_str)
    return [tuple(map(int, coord)) for coord in path_coords]

### --- Z-Layer Offset --- ###
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

### --- Step 4: Plot using Plotly --- ###
def plot_3d_graph(edges, block_positions, path=None):
    fig = go.Figure()

    # Adjust all positions
    graph_nodes = set([pt for edge in edges for pt in edge])
    adjusted_nodes = apply_z_layer_offset(graph_nodes)
    adjusted_blocks = apply_z_layer_offset(block_positions)
    adjusted_path = apply_z_layer_offset(path) if path else []

    # --- Batch edges into a single line trace for performance ---
    edge_x, edge_y, edge_z = [], [], []
    for src, dest in edges:
        p1, p2 = apply_z_layer_offset([src, dest])
        edge_x += [p1[0], p2[0], None]
        edge_y += [p1[1], p2[1], None]
        edge_z += [p1[2], p2[2], None]

    fig.add_trace(go.Scatter3d(
        x=edge_x, y=edge_y, z=edge_z,
        mode='lines',
        line=dict(color='skyblue', width=2),
        name='Edges',
        hoverinfo='none'
    ))

    # --- Graph Nodes ---
    if adjusted_nodes:
        gx, gy, gz = zip(*adjusted_nodes)
        fig.add_trace(go.Scatter3d(
            x=gx, y=gy, z=gz,
            mode='markers',
            marker=dict(size=3, color='crimson'),
            name='Graph Nodes'
        ))

    # --- XML Blocks ---
    if adjusted_blocks:
        bx, by, bz = zip(*adjusted_blocks)
        fig.add_trace(go.Scatter3d(
            x=bx, y=by, z=bz,
            mode='markers',
            marker=dict(size=6, color='limegreen', symbol='square'),
            name='XML Blocks'
        ))

    # --- Path Highlight ---
    if adjusted_path:
        px, py, pz = zip(*adjusted_path)
        fig.add_trace(go.Scatter3d(
            x=px, y=py, z=pz,
            mode='lines+markers',
            line=dict(color='gold', width=6),
            marker=dict(size=6, color='gold', line=dict(width=1, color='black')),
            name='Path'
        ))

    # --- Layout ---
    fig.update_layout(
        title='3D Graph Visualization (Optimized Plotly)',
        scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z',
            aspectmode='data',
        ),
        legend=dict(x=0.02, y=0.98),
        margin=dict(l=0, r=0, b=0, t=30),
        showlegend=True
    )

    fig.show()

### --- Main --- ###
if __name__ == "__main__":
    graph_file = "applicationsBin/AstarMMmvt/graph_edges.txt"
    xml_file = "applicationsBin/AstarMMmvt/Jad_cube10.xml"

    edges = parse_graph_edges(graph_file)
    blocks = parse_block_positions(xml_file)
    path = parse_path(graph_file)
    plot_3d_graph(edges, blocks, path)
