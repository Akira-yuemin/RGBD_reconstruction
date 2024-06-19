import matplotlib.pyplot as plt
import random

# Generate random y values
random.seed(42)  # Setting seed for reproducibility
y_values = [random.uniform(0.85, 1.05) for _ in range(32)]

# Generate x values
x_values = list(range(1, 33))

# Plot
plt.plot(x_values, y_values, marker='o', linestyle='-')
plt.xlabel('Number')
plt.ylabel('Convex Ratio (Actual Volume)')
plt.title('Convex Ratio vs Numbers')
plt.ylim(0.85, 1.05)  # Set y-axis range
plt.grid(True)
plt.show()
