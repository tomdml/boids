import math
from random import randint, uniform

import utils
from config import Config
from vector import Vector2D


class Boid():
    def __init__(self, pos=None, vel=None, color=None):
        self._pos = pos or Vector2D(
            randint(0, Config.SCREEN_X),
            randint(0, Config.SCREEN_Y))

        angle = uniform(-math.pi, math.pi)
        self.vel = vel or Vector2D(
            Config.MAX_SPEED * math.cos(angle),
            Config.MAX_SPEED * math.sin(angle))

        self.color = color or (
            randint(99, 150),
            randint(99, 255),
            randint(200, 255))

        self.turn = 0

    @property
    def pos(self):
        return self._pos

    @pos.setter
    def pos(self, new):
        self._pos = new % (Config.SCREEN_X, Config.SCREEN_Y)

    def step(self, dt):
        self.pos += self.vel * (dt / 1000)  # dt in milliseconds
        self.vel = self.vel.rotate(self.turn * dt / 3)
        self.turn = 0

    def approx_dist(self, other):
        x, y = (other.pos - self.pos)
        if x / Config.SCREEN_X > 0.5:
            x = Config.SCREEN_X - x
        if y / Config.SCREEN_Y > 0.5:
            y = Config.SCREEN_Y - y
        # Reference: https://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
        return (1007 / 1024 * max(abs(x), abs(y))) + (441 / 1024 * min(abs(x), abs(y)))

    def can_see(self, other):
        # Short-circuit here if outside of view radius. Don't bother using sqrt for optimization.
        if self.approx_dist(other) > Config.VIEW_RADIUS:
            return False

        # Check FOV
        selfHeading = self.vel.direction
        relativeHeading = (other.pos - self.pos).direction
        fov = math.radians(Config.FOV)
        angleToOther = utils.neg_mod(selfHeading - relativeHeading, math.pi)
        return angleToOther < (fov / 2)

    def is_close(self, other):
        return self.approx_dist(other) > Config.VIEW_RADIUS / 3

    def turn_by(self, rad):
        if abs(rad) > 0.05:
            rad = math.copysign(0.05, rad)
        self.turn += rad

    def turn_to(self, otherPos, magnitude=10):
        angle = (otherPos - self.pos).direction
        error = utils.neg_mod(angle - self.vel.direction, math.pi)
        self.turn_by(magnitude * error / 1000)

    # Display functions

    def get_line_points(self):
        front = self.pos + Vector2D.from_scalars(self.vel.direction, 20)
        return (front, self.pos)

    def get_arrow_points(self):
        front = self.pos + Vector2D.from_scalars(self.vel.direction, 10)
        left = self.pos + Vector2D.from_scalars(self.vel.direction + (5 / 4 * math.pi), 5)
        right = self.pos + Vector2D.from_scalars(self.vel.direction + (-5 / 4 * math.pi), 5)
        return (front, left, self.pos, right)

    def get_fov_arc_points(self):
        left = self.vel.direction + math.radians(Config.FOV / 2)
        right = self.vel.direction - math.radians(Config.FOV / 2)
        return ((self.pos.x - Config.VIEW_RADIUS, self.pos.y - Config.VIEW_RADIUS, Config.VIEW_RADIUS * 2, Config.VIEW_RADIUS * 2), -left, -right)

    def get_fov_line_points(self):
        left = self.pos + Vector2D.from_scalars(self.vel.direction + (math.radians(Config.FOV / 2)), Config.VIEW_RADIUS)
        right = self.pos + Vector2D.from_scalars(self.vel.direction - (math.radians(Config.FOV / 2)), Config.VIEW_RADIUS)
        return (left, self.pos, right)

    def __str__(self):
        return f"Boid(Pos: {self.pos}, Vel: {self.vel})"
