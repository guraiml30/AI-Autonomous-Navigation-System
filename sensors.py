"""
sensors.py - simulated 360 degree LiDAR.

A real LiDAR returns "distance to the first thing hit" for every beam angle.
We reproduce that with ray marching on the hidden ground-truth world and add
Gaussian noise so the algorithms must cope with imperfect data.
"""
from dataclasses import dataclass
import numpy as np
from . import config as C


@dataclass
class Scan:
    angles_rel: np.ndarray   # beam angles relative to robot heading, [-pi, pi)
    ranges: np.ndarray       # measured distance per beam (m)
    hit: np.ndarray          # True where the beam hit something within range


class Lidar:
    def __init__(self, n_rays=C.LIDAR_RAYS, max_range=C.LIDAR_RANGE,
                 noise=C.LIDAR_NOISE, seed=0):
        self.max_range = max_range
        self.noise = noise
        self.rel = np.linspace(-np.pi, np.pi, n_rays, endpoint=False)
        self.rng = np.random.default_rng(seed)

    def scan(self, world, pose) -> Scan:
        x, y, th = pose
        ranges, hit = world.raycast(x, y, th + self.rel, self.max_range)
        noisy = ranges + np.where(hit, self.rng.normal(0, self.noise, ranges.shape), 0.0)
        noisy = np.clip(noisy, 0.05, self.max_range)
        return Scan(self.rel.copy(), noisy, hit)
