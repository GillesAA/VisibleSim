import matplotlib.pyplot as plt

# Data
x = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6]
# Updated Data
y1_new = [0.529, 0.579, 0.588, 0.612, 0.632, 0.663, 0.686, 0.715, 0.731, 0.751, 0.772, 0.799]
y2_new = [0.444, 0.539, 0.669, 0.673, 0.671, 0.644, 0.616, 0.611, 0.619, 0.637, 0.668, 0.708]
y3_new = [0.583, 0.583, 0.563, 0.5167, 0.498, 0.515, 0.551, 0.593, 0.638, 0.688, 0.737, 0.787]

# Re-plotting with adjusted colors for better accuracy
plt.figure(figsize=(12, 8))  # Larger figure size

line_width = 2.5  # Thicker lines for better visibility
font_size = 14  # Adjusted font size for better readability
marker_size = 10  # Larger markers for clarity

# Adjusting colors to match expectations more closely
# Using 'tab:blue' and 'tab:green' for a more standardized color scheme
plt.plot(x, y1_new, label="Scaffold-1", marker='^', linewidth=line_width, markersize=marker_size, color='tab:blue')
plt.plot(x, y2_new, label="Scaffold-2", marker='x', linewidth=line_width, markersize=marker_size, color='orange')
plt.plot(x, y3_new, label="Scaffold-3", marker='o', linewidth=line_width, markersize=marker_size, color='tab:green')

# Adding titles and labels
plt.ylabel("Probability of Disconnected Graphs Containing 1 Module", fontsize=font_size)
plt.xlabel("Percentage of Failed Connections between 3D-Catoms", fontsize=font_size)

# Adjusting ticks for better visibility
plt.xticks(fontsize=font_size)
plt.yticks(fontsize=font_size)

# Enhancing the legend for clarity
plt.legend(fontsize=font_size)

# Enabling grid for better data interpretation
plt.grid(True)

# Optionally, setting a tight layout
plt.tight_layout()

# Display the plot with adjusted colors
plt.show()


