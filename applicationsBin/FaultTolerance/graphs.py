# Assuming the provided data has been loaded into a DataFrame named `df`

import matplotlib.pyplot as plt
import pandas as pd

# Sample data
data = {

    "Name": ["Scaffold-1"]*12 + ["Scaffold-2"]*12 + ["Scaffold-3"]*12,

    "Percentage": [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6]*3,

    "Normalized Average": [

        0.9817241379310345, 0.9531226053639846, 0.9143358876117497, 0.8280842911877394, 

        0.6867752234993615, 0.49486590038314177, 0.20669220945083014, 0.08653256704980843, 

        0.04826309067688378, 0.025191570881226054, 0.019770114942528737, 0.01739463601532567,

        0.9999925317401046, 0.9997684839432412, 0.9989693801344286, 0.9968483943241224, 

        0.9823300970873786, 0.9794509697422319, 0.9245166803969728, 0.8658228041029705, 

        0.7762244834453572, 0.60526356733881, 0.27992468180445873, 0.10234121263699046,

        0.9978857142857143, 0.9900761904761904, 0.9559190476190477, 0.8984333333333333, 

        0.8468446368446368, 0.7157571428571429, 0.4362244897959184, 0.09252822778595973, 

        0.02688449848024316, 0.010347222222222223, 0.005054112554112554, 0.0035007849293563578

    ],

    "Normalized Std Dev": [

        0.0054997629218424505, 0.012666498760096621, 0.022941581938766626, 0.12483125786905702, 

        0.2085527894320867, 0.2200745285245265, 0.19438335401445886, 0.09631998182116927, 

        0.05343290144952452, 0.01639092695499803, 0.010722608201678077, 0.009816658403703538,

        7.468259895444354e-05, 0.0011948037071935042, 0.002052224430721927, 0.0032967495071900823, 

        0.09901385559247401, 0.013505113154693797, 0.19311657070264973, 0.2250228863801753, 

        0.2535092889776231, 0.294236487757078, 0.24270432414442497, 0.12830060067715052,

        0.0014121551560145136, 0.003433671584762484, 0.13691125115850025, 0.20695398437333662, 

        0.19629836761171468, 0.24044360972553158, 0.2624251825045724, 0.13508251308090527, 

        0.039754731970359766, 0.012132553329796411, 0.004989854339278629, 0.004228527114009196

    ]

}

df = pd.DataFrame(data)
# Example verification code
df['Mean+Std'] = df['Normalized Average'] + df['Normalized Std Dev']
print(df[['Name', 'Percentage', 'Normalized Average', 'Normalized Std Dev', 'Mean+Std']])


# Plotting enhancements
plt.figure(figsize=(16, 10))  # Larger figure size for better detail

# Distinguishable and clear colors
colors = {'Scaffold-1': 'blue', 'Scaffold-2': 'orange', 'Scaffold-3': 'green'}

for scaffold in df['Name'].unique():
    scaffold_data = df[df['Name'] == scaffold]
    plt.errorbar(scaffold_data['Percentage'], scaffold_data['Normalized Average'], 
                 yerr=scaffold_data['Normalized Std Dev'], label=scaffold, fmt='-o', capsize=10, 
                 capthick=2, elinewidth=2, linewidth=2.5, markersize=10, color=colors[scaffold])

# Title and labels with increased font sizes
# plt.title('Percentage of modules reached', fontsize=20)
plt.xlabel('Percentage Faulty connections', fontsize=25)
plt.ylabel('Percentage of modules reached', fontsize=25)

# Ticks
plt.xticks(df['Percentage'].unique(), fontsize=25)
plt.yticks(fontsize=25)

# Legend
plt.legend(fontsize=25)

# Grid
plt.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)

# Axis limits (optional, adjust as needed based on your data)
plt.ylim([0, max(df['Mean+Std']) * 1.1])  # Adjusting Y limit based on max mean+std

# Tight layout for better spacing
plt.tight_layout()

# High-resolution save for publication
plt.savefig('normalized_module_data_high_res.png', dpi=300)

plt.show()