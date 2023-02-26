# Author: Grigorev Timofey

import math


class Vec2d:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "x: " + str(self.x) + " y: " + str(self.y)

    def get_length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def to_unit_vec(self):
        l = self.get_length()
        if l:
            self.x /= l
            self.y /= l

    def dot_product(self, other):
        return self.x * other.x + self.y * other.y

    def rotate(self, angle):
        cos_final = self.x * math.cos(angle) - self.y * math.sin(angle)
        sin_final = self.x * math.sin(angle) + self.y * math.cos(angle)
        self.x = cos_final
        self.y = sin_final


class Line:

    def __init__(self, x1, y1, x2, y2):
        self.coords = [x1, y1, x2, y2]
        self.ka = 0
        self.kb = 0
        self.kc = 0

    def __str__(self):  # for debugging only!
        return str(self.ka) + " " + str(self.kb) + " " + str(self.kc)

    def calc_coeffs(self):
        self.ka = self.coords[-1] - self.coords[1]
        self.kb = self.coords[0] - self.coords[2]
        self.kc = self.coords[1] * self.coords[2] - self.coords[0] * self.coords[-1]

    def get_intersection(self, other):
        check_term = self.kb * other.ka - self.ka * other.kb
        if check_term:
            x_intersect = (self.kc * other.kb - self.kb * other.kc) / check_term
            y_intersect = 0
            if self.kb:
                y_intersect = -(self.ka * x_intersect + self.kc) / self.kb
            else:
                y_intersect = -(other.ka * x_intersect + other.kc) / other.kb
            return x_intersect, y_intersect
        return None, None


class GenericBody:

    def __init__(self, x_conv, y_conv, x, y, k, m):
        self.conv = [x_conv, y_conv]
        self.pos = [x, y]
        self.k = k  # friction coefficient
        self.m = m  # mass in SI
        self.f = Vec2d(0, 0)  # some artificial force
        self.v = Vec2d(0, 0)
        self.a = Vec2d(0, 0)

    def get_accel(self):
        # Calculating all forces
        friction = self.k * self.m * 9.87
        v_t = math.sqrt(self.v.x ** 2 + self.v.y ** 2)
        cos, sin = 0, 0
        if v_t != 0:
            cos = -self.v.x / v_t
            sin = -self.v.y / v_t
        r_x = self.f.x + friction * cos
        r_y = self.f.y + friction * sin
        # Calculating acceleration and velocity
        a_x = r_x / self.m
        a_y = r_y / self.m
        accel = Vec2d(a_x, a_y)
        return accel

    def update_position(self, f_time, p_x=None, p_y=None, r_ang=None):
        a = self.get_accel()
        if self.v.x + a.x * f_time < 0 and self.f.x == 0 and a.x < 0:
            self.v.x = 0
        else:
            self.v.x += a.x * f_time
        if self.v.y + a.y * f_time < 0 and self.f.y == 0 and a.y < 0:
            self.v.y = 0
        else:
            self.v.y += a.y * f_time
        # Calculating how many meters the body has travelled from its previous position
        trv_x = self.v.x * f_time
        trv_y = self.v.y * f_time
        # Updating the position
        # Check if the body will reach predicted collision point
        x_check, y_check = 0, 0
        if p_x is not None and p_y is not None:
            # Is the predicted collision point between current and future positions?
            x_check = (self.pos[0] + trv_x - p_x) * (self.pos[0] - p_x)
            y_check = (self.pos[1] + trv_y - p_y) * (self.pos[1] - p_y)
        if x_check < 0 or y_check < 0:
            excess_dist = Vec2d(trv_x + self.pos[0] - p_x, trv_y + self.pos[1] - p_y)
            excess_dist.rotate(r_ang)
            self.v.rotate(r_ang)
            self.pos[0] = p_x + excess_dist.x
            self.pos[1] = p_y + excess_dist.y
            return True
        else:
            self.pos[0] += trv_x
            self.pos[1] += trv_y
            return False


class CircularBody(GenericBody):

    def __init__(self, x_conv, y_conv, x, y, k, m, r):
        super().__init__(x_conv, y_conv, x, y, k, m)
        self.circ_collision = False
        self.r = r

    def has_collided_circ(self, other):
        r1 = self.r
        r2 = other.r
        dist = math.sqrt((self.pos[0] - other.pos[0]) ** 2 + (self.pos[1] - other.pos[1]) ** 2)
        return dist <= r1 + r2

    def has_collided_ln(self, x1, y1, x2, y2):
        ln1 = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        ln2 = math.sqrt((self.pos[0] - x1) ** 2 + (self.pos[1] - y1) ** 2)
        ln3 = math.sqrt((self.pos[0] - x2) ** 2 + (self.pos[1] - y2) ** 2)
        p = (ln1 + ln2 + ln3) / 2
        area = math.sqrt(p * (p - ln1) * (p - ln2) * (p - ln3))
        dist = area * 2 / ln1
        return dist <= 0.001

    def update_position(self, f_time, p_x=None, p_y=None, r_ang=None, circ_objs=None):
        a = self.get_accel()
        if self.v.x + a.x * f_time < 0 and self.f.x == 0 and a.x < 0:
            self.v.x = 0
        else:
            self.v.x += a.x * f_time
        if self.v.y + a.y * f_time < 0 and self.f.y == 0 and a.y < 0:
            self.v.y = 0
        else:
            self.v.y += a.y * f_time
        # Calculating how many meters the body has travelled from its previous position
        trv_x = self.v.x * f_time
        trv_y = self.v.y * f_time
        # Updating the position
        # Check if the body will collide with any circular objects
        if circ_objs is not None and not len(circ_objs) % 3:
            n_not_collided = 0
            for i in range(0, len(circ_objs), 3):
                dist = math.sqrt((self.pos[0] + trv_x - circ_objs[i]) ** 2 +
                                 (self.pos[1] + trv_y - circ_objs[i+1]) ** 2)
                if 0 < dist < circ_objs[i+2] + self.r:
                    l_ratio = (circ_objs[i+2] + self.r) / dist - 1
                    x_new = self.pos[0] + (self.pos[0] - circ_objs[i]) * l_ratio
                    y_new = self.pos[1] + (self.pos[1] - circ_objs[i + 1]) * l_ratio
                    if not self.circ_collision:
                        self.v.rotate(math.pi)
                    self.pos[0] = x_new
                    self.pos[1] = y_new
                    self.circ_collision = True
                    return 2
                else:
                    n_not_collided += 1
            if n_not_collided == len(circ_objs) // 3:
                self.circ_collision = False

        # Check if the body will reach predicted collision point
        x_check, y_check = 0, 0
        if p_x is not None and p_y is not None:
            # Is the predicted collision point between current and future positions?
            x_check = (self.pos[0] + trv_x - p_x) * (self.pos[0] - p_x)
            y_check = (self.pos[1] + trv_y - p_y) * (self.pos[1] - p_y)
        if x_check < 0 or y_check < 0:
            excess_dist = Vec2d(trv_x + self.pos[0] - p_x, trv_y + self.pos[1] - p_y)
            excess_dist.rotate(r_ang)
            self.v.rotate(r_ang)
            self.pos[0] = p_x + excess_dist.x
            self.pos[1] = p_y + excess_dist.y
            return 1
        else:
            self.pos[0] += trv_x
            self.pos[1] += trv_y
            return 0
