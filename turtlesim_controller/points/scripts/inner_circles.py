import json
import numpy as np

def generate_circle(radius, center, num_points):
    theta = np.linspace(0, 2*np.pi, num_points)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    return list(zip(x, y))

# Define the center of the torus yantra
center = [5, 5]

# Generate a list of radii for the concentric circles
radii = np.linspace(0.5, 2.5, 5)

# Generate the circles
points = []
for radius in radii:
    circle = generate_circle(radius, center, 100)
    points.extend(circle)

# Scale points to fit within turtlesim's coordinate system [2, 8]
points = np.array(points)
points[:, 0] = 2 + 6 * (points[:, 0] - np.min(points[:, 0])) / (np.max(points[:, 0]) - np.min(points[:, 0]))
points[:, 1] = 2 + 6 * (points[:, 1] - np.min(points[:, 1])) / (np.max(points[:, 1]) - np.min(points[:, 1]))

# Print points
print(json.dumps(points.tolist(), indent = 4))

