import subprocess
import pandas as pd
import numpy as np
import os

def run_cpp_program_and_count_graphs(exclude_roots):
    # Run the C++ program and capture its output
    process = subprocess.Popen(["./GraphNodeDetection", "-t", "-c", "Jad_cube.xml"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output, _ = process.communicate()
    output = output.decode('utf-8')
    
    # Process each line, exclude specified roots, and count the remaining graphs
    graph_count = 0
    for line in output.split('\n'):
        if line.startswith('root:'):
            root = int(line.split()[1].split('->')[0])
            if root not in exclude_roots:
                graph_count += 1
    return graph_count

def perform_iterations(iterations, exclude_roots):
    counts = [run_cpp_program_and_count_graphs(exclude_roots) for _ in range(iterations)]
    return counts

def calculate_statistics(counts):
    average = np.mean(counts)
    std_dev = np.std(counts, ddof=1)  # Using ddof=1 for sample standard deviation
    return average, std_dev

def append_to_csv(name, percentage, average, std_dev, filename):
    df = pd.DataFrame({
        'Name': [name],
        'Percentage': [percentage],
        'Average Graph Count': [average],
        'Standard Deviation': [std_dev]
    })
    
    file_exists = os.path.isfile(filename)
    df.to_csv(filename, mode='a', index=False, header=not file_exists)

if __name__ == "__main__":
    iterations = 100
    exclude_roots = []
    # exclude_roots = [53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298]  # Specify the roots to be excluded
    name = "Scaffold-3"
    percentage = 0.6  # Adjust this value as needed for different runs
    filename = "./graph_counts_results.csv"  # Specify your file path

    counts = perform_iterations(iterations, exclude_roots)
    average, std_dev = calculate_statistics(counts)
    append_to_csv(name, percentage, average, std_dev, filename)

    print(f"Results appended to {filename}")
