# Author: Grigorev Timofey

import physics
import math


def solve_square_equation(a, b, c):
    d = b**2 - 4*a*c
    if d > 0:
        x1, x2 = (-b+math.sqrt(d)) / (2 * a), (-b-math.sqrt(d)) / (2 * a)
        return x1, x2
    elif d == 0:
        return -b/(2 * a), None
    return None, None


def predict_ball_pos(border, curr_vel, curr_accel, curr_pos, path, n_depth=-1):
    collide_coordinates = []
    rotation_angles = []
    run_predict = True
    was_terminated = False
    n_iterations = 0
    while run_predict:
        path.calc_coeffs()
        x_collide, y_collide = 0, 0
        collision_angle = 0
        dist_intc = -1  # last registered distance between intersection point and current position
        for j in border:
            nml = physics.Vec2d(j.ka, j.kb)
            nml.to_unit_vec()
            if curr_vel.dot_product(nml) < 0:
                xi, yi = j.get_intersection(path)
                if xi is not None:
                    dist = math.sqrt((xi - curr_pos[0]) ** 2 + (yi - curr_pos[1]) ** 2)
                    if (dist_intc + 1 and dist < dist_intc) or not dist_intc + 1:
                        x_collide = xi
                        y_collide = yi
                        # Calculate angle between trajectory and the wall that we've hit
                        tmp_vec = physics.Vec2d(curr_vel.x, curr_vel.y)
                        wall_vec = physics.Vec2d(j.coords[0] - j.coords[2], j.coords[1] - j.coords[3])
                        tmp_vec.to_unit_vec()
                        wall_vec.to_unit_vec()
                        collision_angle = math.acos(tmp_vec.dot_product(wall_vec))
                        dist_intc = dist
        # Calculate time to reach collision point
        t = 0
        if curr_accel.x:
            t1, t2 = solve_square_equation(curr_accel.x, 2 * curr_vel.x, -2 * (x_collide - curr_pos[0]))
            if t1 is not None and t1 >= 0 and t2 is None:
                t = t1
            elif t2 is not None and t2 >= 0 and t1 is None:
                t = t2
            elif t1 is not None and t2 is not None and t1 >= 0 and t2 >= 0:
                t = min(t1, t2)
        elif curr_accel.y:
            t1, t2 = solve_square_equation(curr_accel.y, 2 * curr_vel.y, -2 * (y_collide - curr_pos[1]))
            if t1 is not None and t1 >= 0 and t2 is None:
                t = t1
            elif t2 is not None and t2 >= 0 and t1 is None:
                t = t2
            elif t1 is not None and t2 is not None and t1 >= 0 and t2 >= 0:
                t = min(t1, t2)
        # If collision point is unreachable, interpolate
        is_unreachable = t == 0 and curr_pos[0] != x_collide and curr_pos[1] != y_collide
        if is_unreachable or n_iterations == n_depth:
            if not is_unreachable:
                was_terminated = True
            else:
                t = curr_vel.x / -curr_accel.x  # V+a*t=0
                x_reach = (curr_vel.x * t) / 2
                y_reach = (y_collide - curr_pos[1]) * x_reach / (x_collide - curr_pos[0])
                x_collide = x_reach + curr_pos[0]
                y_collide = y_reach + curr_pos[1]
            run_predict = False

        # Modify velocity based on time to reach collision point
        curr_vel.x += curr_accel.x * t
        curr_vel.y += curr_accel.y * t
        curr_vel.rotate(2 * collision_angle)
        curr_accel.rotate(2 * collision_angle)
        curr_pos = [x_collide, y_collide]
        path.coords = [x_collide, y_collide, x_collide + curr_vel.x, y_collide + curr_vel.y]  # Modify trajectory
        collide_coordinates.append(x_collide)
        collide_coordinates.append(y_collide)
        rotation_angles.append(2 * collision_angle)
        n_iterations += 1
    return collide_coordinates, rotation_angles, curr_pos, was_terminated
