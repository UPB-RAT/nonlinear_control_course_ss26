import math


def get_figure8_target(A, w, height, t):
    tx = A * math.sin(w * t)
    ty = 0.5 * A * math.sin(2 * w * t)
    tz = height
    return tx, ty, tz


def get_circle_target(radius, w, height, t):
    tx = radius * math.cos(w * t)
    ty = radius * math.sin(w * t)
    tz = height
    return tx, ty, tz


def get_spiral_target(radius, w, climb_rate, t):
    tx = radius * math.cos(w * t)
    ty = radius * math.sin(w * t)
    tz = climb_rate * t
    return tx, ty, tz


def get_square_target(side, speed, t):

    segment_time = side / speed
    cycle_time = 4 * segment_time
    t = t % cycle_time

    if t < segment_time:
        tx = -side / 2 + speed * t
        ty = -side / 2
    elif t < 2 * segment_time:
        tx = side / 2
        ty = -side / 2 + speed * (t - segment_time)
    elif t < 3 * segment_time:
        tx = side / 2 - speed * (t - 2 * segment_time)
        ty = side / 2
    else:
        tx = -side / 2
        ty = side / 2 - speed * (t - 3 * segment_time)

    tz = 1.0
    return tx, ty, tz


def get_target(traj_type, params, t):

    if traj_type == "figure8":
        return get_figure8_target(
            params["A"], params["w"], params["height"], t
        )

    elif traj_type == "circle":
        return get_circle_target(
            params["radius"], params["w"], params["height"], t
        )

    elif traj_type == "spiral":
        return get_spiral_target(
            params["radius"], params["w"], params["climb_rate"], t
        )

    elif traj_type == "square":
        return get_square_target(
            params["side"], params["speed"], t
        )

    else:
        raise ValueError(f"Unknown trajectory: {traj_type}")