import pandas as pd

# Load the data from the CSV file
df = pd.read_csv('scaffold_data.csv')

# Conversion factors for each scaffold's average reachable modules
conversion_factors = {
    "Scaffold-1": 1566,
    "Scaffold-2": 1339,
    "Scaffold-3": 2100
}

# Function to normalize both 'Average ReachableModules' and 'Standard Deviation' based on the scaffold name
def normalize(row):
    factor = conversion_factors[row['Name']]
    normalized_average = row['Average ReachableModules'] / factor
    normalized_std_dev = row['Standard Deviation'] / factor
    return normalized_average, normalized_std_dev

# Apply normalization
df[['Normalized Average', 'Normalized Std Dev']] = df.apply(lambda row: pd.Series(normalize(row)), axis=1)

# Assuming the DataFrame `df` has already been created from the CSV as shown previously

# Removing the 'Average ReachableModules' and 'Standard Deviation' columns
df.drop(['Average ReachableModules', 'Standard Deviation'], axis=1, inplace=True)

# Saving the modified DataFrame back to the CSV file, overwriting the original
df.to_csv('normalized_data_final.csv', index=False)
