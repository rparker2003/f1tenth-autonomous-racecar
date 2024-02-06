import matplotlib.pyplot as plt
import numpy as np

# Time differences
times_bank1 = [9.91, 10.29, 45.26]
times_bank2 = [5.73, 5.80, 29.51]

# F1Track
# times_bank1 = [44.83, 44.05, 46.89]
# times_bank2 = [29.51, 30.00, 30.01]

# Dumbbell
# times_bank1 = [10.01, 10.16, 9.95, 10.83, 10.14, 10.08, 10.85]
# times_bank2 = [5.99, 5.38, 5.77, 5.88, 5.47, 5.83, 5.95]


# TBE-A Great Hall
# times_bank1 = [10.08, 10.10, 9.55, 9.36, 10.15, 10.05, 10.10] 
# times_bank2 = [6.02, 5.57, 6.00, 5.49, 5.59, 6.05, 5.51]

# Corresponding x values (just an example, you can customize it)
x_values = np.arange(len(times_bank1))+1

y_limit = max(max(times_bank1), max(times_bank2)) + 2

# Plotting the data vertically
bar_width = 0.4  # Width of each bar
bar_shift = 0.2  # Distance to shift bars for side-by-side display

# Plotting the data
plt.bar(x_values - bar_shift, times_bank1, width=bar_width, label='Pure Pursuit', color='blue')
plt.bar(x_values + bar_shift, times_bank2, width=bar_width, label='Raceline Optimization', color='orange')

# Adding a title and legend
plt.title('Lap Times Between Pure Pursuit and Raceline Optimization')

# side_note_text = "Map: F1 Track"
# plt.figtext(0.28, 0.85, side_note_text, ha='center', va='center', fontsize=12)

plt.legend(loc='best')

# Adding labels to the axes (optional)
# plt.xlabel('Lap Number')
plt.ylabel('Lap Times (Seconds)')

plt.ylim(0, y_limit)

# Save the plot as an image file
plt.savefig('inverted_bar_plot.png')
