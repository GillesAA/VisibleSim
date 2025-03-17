import subprocess
import re
import os
import pandas as pd
import numpy as np

def run_cpp_program():
    # Run the C++ program and capture its output
    process = subprocess.Popen(["./GraphNodeDetection", "-t", "-c", "Jad_cube.xml"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    output, _ = process.communicate()

    # Decode the output to a string for further processing
    output = output.decode('utf-8')
    return output

def parse_and_aggregate_outputs(iterations, exclude_roots):
    aggregate_results = {}
    for _ in range(iterations):
        output = run_cpp_program()
        pattern = re.compile(r"root: (\d+)->(\d+)")
        for line in output.split('\n'):
            match = pattern.match(line)
            if match:
                root, size = int(match.group(1)), int(match.group(2))
                if root not in exclude_roots:
                    if root not in aggregate_results:
                        aggregate_results[root] = []
                    aggregate_results[root].append(size)
    return aggregate_results

def compute_statistics(graph_sizes):
    # Compute average and standard deviation for sizes of each graph
    statistics = []
    for root, sizes in graph_sizes.items():
        average_size = np.mean(sizes)
        std_dev = np.std(sizes, ddof=1)  # Using ddof=1 for sample standard deviation
        statistics.append({'Graph Root': root, 'Average Size': average_size, 'Standard Deviation': std_dev})
    return statistics

def append_to_csv(statistics, name, percentage, filename):
    df = pd.DataFrame(statistics)
    df['Name'] = name
    df['Percentage'] = percentage
    
    # Check if the file exists to determine if the header should be written
    file_exists = os.path.isfile(filename)
    # Append to CSV, writing header only if the file doesn't exist
    df.to_csv(filename, mode='a', index=False, header=not file_exists)



if __name__ == "__main__":
    iterations = 100
    exclude_roots = []
    # exclude_roots = [53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254, 255, 256, 257, 258, 259, 260, 261, 262, 263, 264, 265, 266, 267, 268, 269, 270, 271, 272, 273, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298]  # Specify the roots to be excluded
    percentage = 0.6  # Example, adjust as needed
    name = "Scaffold-3"
    # Specify the CSV file path
    filename = "./graph_table.csv"  # Adjust the path accordingly

    aggregated_outputs = parse_and_aggregate_outputs(iterations, exclude_roots)
    statistics = compute_statistics(aggregated_outputs)
    append_to_csv(statistics, name, percentage, filename)

    print(f"Results appended to {filename}")
