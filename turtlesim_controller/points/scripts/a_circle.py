import math
import json

# Outer circle
points = [
    [5, 8],
    [6.5, 7.5],
    [7.5, 6.5],
    [8, 5],
    [7.5, 3.5],
    [6.5, 2.5],
    [5, 2],
    [3.5, 2.5],
    [2.5, 3.5],
    [2, 5],
    [2.5, 6.5],
    [3.5, 7.5],
    [5, 8]
]

# Inner spiral
for r in range(6, 0, -1):
    for a in range(0, 360, 30):
        x = 5 + r * 0.25 * math.cos(a * math.pi / 180)
        y = 5 + r * 0.25 * math.sin(a * math.pi / 180)
        points.append([round(x, 1), round(y, 1)])


print(json.dumps(points, indent=4))

