# Author: Grigorev Timofey

import main_logic
import physics
import time
import threading
import pygame as pg
from pygame.locals import*

# Physics setup

ball_radius = 0.1
field_width = 3
field_height = 2
f_time = 1

border = [physics.Line(field_width - ball_radius, ball_radius, ball_radius, ball_radius),
          physics.Line(field_width - ball_radius, field_height - ball_radius, field_width - ball_radius, ball_radius),
          physics.Line(ball_radius, field_height - ball_radius, field_width - ball_radius, field_height - ball_radius),
          physics.Line(ball_radius, ball_radius, ball_radius, field_height - ball_radius)]

for i in border:
    i.calc_coeffs()

handle_l = physics.CircularBody(field_width, field_height, 0.4, 1, 0.05, 0.01, 0.2)
ball = physics.CircularBody(field_width, field_height, 1.5, 1, 0.04, 0.005, 0.1)
ball.v.x = 8.7
ball.v.y = 4.7

# Position estimation

curr_v = physics.Vec2d(ball.v.x, ball.v.y)
curr_accel = ball.get_accel()
curr_pos = ball.pos
curr_path = physics.Line(ball.pos[0], ball.pos[1], ball.pos[0] + curr_v.x, ball.pos[1] + curr_v.y)
collision_idx = 2
rot_idx = 0
n_predict_depth = 3

# Generic GUI things

w, h = 640, 480
norm_spd = 0.8
pg.init()
screen = pg.display.set_mode((w, h), pg.RESIZABLE)
pg.display.set_caption("Aer0_h0ck3y")
running = True
collided = False  #
sysfont = pg.font.get_default_font()
font = pg.font.SysFont(None, 48)

# test

draw_trajectory = False
collision_point, rotation_angles, curr_pos, gen_path = main_logic.predict_ball_pos(border, curr_v, curr_accel, curr_pos, curr_path)
print("Predicted collisions:", collision_point)
collision_past = [collision_point[0], collision_point[1]]

while running:
    t1 = time.perf_counter()
    screen.fill((0, 0, 0))
    pg.draw.circle(screen, (255, 255, 255), (handle_l.pos[0] * w / handle_l.conv[0], h - (handle_l.pos[1] * h / handle_l.conv[1])), handle_l.r * min(w, h) / 2)
    pg.draw.circle(screen, (255, 255, 255), (ball.pos[0] * w / ball.conv[0], h - (ball.pos[1] * h / ball.conv[1])), ball.r * min(w, h) / 2)
    if draw_trajectory:
        for i in range(2, len(collision_point), 2):
            s_x, s_y = collision_point[i - 2] * w / ball.conv[0], h - (collision_point[i - 1] * h / ball.conv[1])
            e_x, e_y = collision_point[i] * w / ball.conv[0], h - (collision_point[i + 1] * h / ball.conv[1])
            pg.draw.line(screen, (255, 0, 0), (s_x, s_y),
                         (e_x, e_y))
            pg.draw.circle(screen, (255, 0, 255), (s_x, s_y), 4)
            pg.draw.circle(screen, (255, 0, 255), (e_x, e_y), 4)
    img = font.render(str(((1 / f_time) // 10) * 10), True, (255, 0, 0))
    screen.blit(img, (0, 0))
    pg.display.update()
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
        elif event.type == KEYDOWN:
            if event.key == K_a:
                handle_l.v.x += -norm_spd
            elif event.key == K_d:
                handle_l.v.x += norm_spd
            elif event.key == K_w:
                handle_l.v.y += norm_spd
            elif event.key == K_s:
                handle_l.v.y += -norm_spd
            elif event.key == K_x:
                draw_trajectory = not draw_trajectory
        elif event.type == pg.VIDEORESIZE:
            w, h = event.w, event.h
            screen = pg.display.set_mode((w, h), pg.RESIZABLE)
    t2 = time.perf_counter()

    f_time = t2 - t1
    fps = 1 / f_time

    if gen_path:
        tmp_points, tmp_angles, curr_pos, gen_path = main_logic.predict_ball_pos(border, curr_v, curr_accel,
                                                                                           curr_pos, curr_path, n_predict_depth)
        for i in range(len(tmp_angles)):
            collision_point.append(tmp_points[i * 2])
            collision_point.append(tmp_points[i * 2 + 1])
            rotation_angles.append(tmp_angles[i])
        collision_past[0] = collision_point[0]
        collision_past[1] = collision_point[1]

    handle_l.update_position(f_time)
    has_collided = ball.update_position(f_time, collision_past[0], collision_past[1],
                                        rotation_angles[rot_idx], [handle_l.pos[0], handle_l.pos[1],
                                                                   handle_l.r])
    if has_collided == 1:
        collision_past[0] = collision_point[collision_idx]
        collision_past[1] = collision_point[collision_idx + 1]
        collision_idx += 2
        rot_idx += 1
    elif has_collided == 2:
        collision_point.clear()
        rotation_angles.clear()
        ball.v.x += handle_l.v.x
        ball.v.y += handle_l.v.y
        ball.a.x += handle_l.a.x
        ball.a.y += handle_l.a.y
        handle_l.v.x, handle_l.v.y = 0, 0
        curr_v = physics.Vec2d(ball.v.x, ball.v.y)
        curr_accel = ball.get_accel()
        curr_pos = ball.pos
        curr_path.coords = [ball.pos[0], ball.pos[1], ball.pos[0] + curr_v.x, ball.pos[1] + curr_v.y]
        gen_path = True
        collision_idx, rot_idx = 2, 0

pg.quit()
