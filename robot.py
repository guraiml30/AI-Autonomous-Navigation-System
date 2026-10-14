"""robot.py - kinematic model of the virtual robot (unicycle / differential drive)."""
import math
from . import config as C


class Robot:
    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta
        self.v = 0.0     # linear speed (m/s), negative = reversing
        self.w = 0.0     # angular speed (rad/s)

    @property
    def pose(self):
        return (self.x, self.y, self.theta)

    def step(self, v_cmd, w_cmd, dt=C.DT):
        # Speed changes gradually (acceleration limit) but braking is stronger.
        if v_cmd > self.v:
            self.v = min(v_cmd, self.v + C.ACCEL)
        else:
            self.v = max(v_cmd, self.v - C.BRAKE)
        self.w = 0.6 * self.w + 0.4 * w_cmd                 # smooth steering
        self.theta = (self.theta + self.w * dt + math.pi) % (2 * math.pi) - math.pi
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
