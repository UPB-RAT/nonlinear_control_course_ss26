import numpy as np
import json

def cayley(u, v):
    X = u**3 - v**3
    Y = u**2*v - v**2*u
    return X, Y

# Generate evenly spaced values for u and v between -1 and 1
u_values = np.linspace(-1, 1, 200)
v_values = np.linspace(-1, 1, 200)

# Generate points for the Cayley's nodal cubic surface
points = []
for u in u_values:
    for v in v_values:
        X, Y = cayley(u, v)
        points.append([X, Y])

# Convert points to a numpy array for easier manipulation
points = np.array(points)

# Compute min and max of X and Y for scaling
X_min, Y_min = np.min(points, axis=0)
X_max, Y_max = np.max(points, axis=0)

# Scale X and Y to match the turtlesim's coordinate system [2, 8]
points[:, 0] = 2 + 6 * (points[:, 0] - X_min) / (X_max - X_min)
points[:, 1] = 2 + 6 * (points[:, 1] - Y_min) / (Y_max - Y_min)

# Define the size of the grid for downsampling
grid_size = 50  # Adjust this for more or less points

# Create a 2D grid to hold the downsampled points
grid = np.full((grid_size, grid_size, 2), np.nan)

# Assign each point to a cell in the grid
for point in points:
    # Compute the cell coordinates in the grid for this point
    i = int((point[0] - 2) / 6 * (grid_size - 1))
    j = int((point[1] - 2) / 6 * (grid_size - 1))

    # Store this point in the corresponding cell
    grid[i, j] = point

# Convert the grid back to a list of points, ignoring empty cells
downsampled_points = [point.tolist() for point in grid.reshape(-1, 2) if not np.isnan(point).any()]

# Print points
print(json.dumps(downsampled_points, indent=4))
