import pandas as pd
import matplotlib.pyplot as plt

# Load the data from the provided file
file_path = 'state.csv'
data = pd.read_csv(file_path, delimiter=' ')

# Drop any unnecessary columns (in this case, 'Unnamed: 5' which is NaN)
# data = data.drop(columns=['Unnamed: 5'])

# Define the target point
target_x, target_y = -1, -2

# Plotting Y vs X to represent the robot's trajectory along with the target point
plt.figure(figsize=(10, 6))
plt.plot(data['x'], data['y'], color='b', linewidth=2, label='Траектория робота')
plt.scatter(target_x, target_y, color='r', s=300, marker='*', label='Целевая точка (-1, -2)')
plt.xlabel('X, м', fontsize=25)
plt.ylabel('Y, м', fontsize=25)
plt.title('', fontsize=16)
plt.legend(fontsize=18)
plt.grid(True)
plt.show()
