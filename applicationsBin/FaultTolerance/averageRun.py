import subprocess
import re
import pandas as pd
import numpy as np
import os

def run_fault_tolerance_command(iterations, name, percentage, filename):
    pattern = re.compile(r"ReachableModules: (\d+)")
    reachable_modules_values = []

    for _ in range(iterations):
        # Run the FaultTolerance command and capture the output
        process = subprocess.Popen(["./FaultTolerance", "-t", "-c", "Jad_cube.xml"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, _ = process.communicate()
        output = stdout.decode('utf-8')  # Decode the output

        # Find all occurrences of "ReachableModules" and their values
        matches = pattern.findall(output)
        if matches:
            # Convert the last occurrence to int and append to the list
            last_value = int(matches[-1])
            reachable_modules_values.append(last_value)

    # Calculate the average and standard deviation of the ReachableModules values
    if reachable_modules_values:
        average = np.mean(reachable_modules_values)
        std_dev = np.std(reachable_modules_values, ddof=1)  # Using ddof=1 for sample standard deviation
        append_to_csv(name, percentage, average, std_dev, filename)
    else:
        print("No ReachableModules values found in the output.")

def append_to_csv(name, percentage, average, std_dev, filename):
    df = pd.DataFrame({
        'Name': [name],
        'Percentage': [percentage],
        'Average ReachableModules': [average],
        'Standard Deviation': [std_dev]
    })
    
    file_exists = os.path.isfile(filename)
    df.to_csv(filename, mode='a', index=False, header=not file_exists)

if __name__ == "__main__":
    name = "Scaffold-3"  # Example name, adjust as needed
    percentage = 0.6  # Example percentage, adjust as needed
    filename = "./dc.csv"  # Specify your file path
    iterations = 100
    
    run_fault_tolerance_command(iterations, name, percentage, filename)
    print(f"Results appended to {filename}")
