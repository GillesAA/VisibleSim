import matplotlib.pyplot as plt
import pandas as pd

# Data
data = {
    "Name": ["Scaffold-1"]*12 + ["Scaffold-2"]*12 + ["Scaffold-3"]*12,
    "Percentage": [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6]*3,
    "Average Graph Count": [
        39.44, 95.66, 165.02, 245.6, 328.88, 416.84, 512.97, 609.9, 723.7, 841.13, 959.7, 1076.83,
        1.2, 3.0, 8.95, 22.92, 49.5, 95.59, 165.68, 257.51, 371.27, 501.38, 636.9, 768.6,
        12.09, 50.64, 127.21, 256.0, 431.29, 616.85, 806.01, 989.2, 1159.07, 1321.41, 1464.38, 1595.18
    ],
    "Standard Deviation": [
        5.41289145830525, 9.190706768746946, 10.099484935361602, 12.295190814774578, 11.849459775436895,
        12.616103225880035, 13.582114650666089, 14.314293202913998, 12.970440363855163, 13.308047994132776,
        11.61286294841138, 13.24062847542527,
        0.4714045207910317, 1.3926212476455828, 2.750114781810263, 4.282168807621869, 7.0760657904450746,
        8.075521060838144, 12.067888771195207, 10.857660331709951, 11.314471827145654, 14.546185587689823,
        13.531482184188476, 14.395566096278507,
        3.1689624851555487, 6.072824048100844, 8.539031005040941, 10.938764356935643, 12.31103225369486,
        11.620579967793294, 12.973551183196923, 13.170981264687438, 13.640592273642579, 14.55542871407452,
        12.313070988657426, 13.30404358672865
    ]
}

df = pd.DataFrame(data)

# Enhancements for visibility
fig, ax = plt.subplots(figsize=(12, 8))  # Bigger figure size

# Distinguishable and clear colors
colors = {'Scaffold-1': 'blue', 'Scaffold-2': 'orange', 'Scaffold-3': 'green'}

for name, group in df.groupby('Name'):
    ax.errorbar(group['Percentage'], group['Average Graph Count'], yerr=group['Standard Deviation'],
                fmt='o-', capsize=10, label=name, color=colors[name],
                linewidth=2, markersize=6, markeredgewidth=2, elinewidth=2)  # Thicker error bars, smaller markers

ax.set_xlabel('Percentage of Failed Connections', fontsize=22)
ax.set_ylabel('Average Disconnected Graph Count', fontsize=22)
ax.tick_params(axis='both', which='major', labelsize=22)
ax.grid(True, linestyle='--', linewidth=0.5)  # Light grid
ax.legend(fontsize=22)

plt.tight_layout()
plt.show()
