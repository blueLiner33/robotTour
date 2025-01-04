import random
import pandas as pd

# Parameters
num_data_points = 150

# Initialize dataset
data = []

# Yaw behavior setup (right, pause, left, pause, left)
yaw_states = [(90, 'right'), (90, 'pause'), (-90, 'left'), (-90, 'pause'), (-90, 'left')]
yaw_changes = []
for state in yaw_states:
    yaw_changes.extend([state[0]] * (num_data_points // len(yaw_states)))

# Generate data
current_yaw = 0
for i in range(num_data_points):
    # Gradual yaw changes
    yaw_change = yaw_changes[i]
    if yaw_change != 0:
        current_yaw += yaw_change / (num_data_points // len(yaw_states))
        current_yaw = round(current_yaw,2)

    # Generate random motor RPMs and degrees
    rpm1 = round(random.uniform(0, 500), 2)
    rpm2 = round(random.uniform(0, 500), 2)
    degrees_one = round((i * 1.5) % 360, 2)  # simulate degrees count (cyclic)
    degrees_two = round((i * 1.3) % 360, 2)

    # Static accelerations
    x_accel = 0
    y_accel = 0
    z_accel = 0

    # Append to dataset
    data.append([rpm1, degrees_one, rpm2, degrees_two, current_yaw, x_accel, y_accel, z_accel])

print(data)