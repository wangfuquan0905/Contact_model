# -*- coding: utf-8 -*-
"""Penalty-based collision:
For each overlapping pair, apply spring-damper force along the contact normal.
Also handle wall-penetrations as penalty contacts.
- k: stiffness (N/m)
- c: damping (N*s/m) on the normal relative speed
"""
from .base import CollisionModel
import numpy as np
import math

class PenaltyCollision(CollisionModel):
    name = "penalty"
    def __init__(self, k: float = 800.0, c: float = 8.0, e_target: float = 0.9):
        self.k = float(k)
        self.c = float(c)
        self.e_target = float(e_target)

    def apply(self, world, h: float):
        # Pairwise sphere penalties
        for i, j, a, b in world.each_pairs():
            dx = b.x - a.x
            dist2 = float(dx @ dx)
            rad = a.r + b.r
            if dist2 >= rad*rad:
                continue
            dist = math.sqrt(dist2) if dist2 > 1e-12 else 0.0
            if dist > 1e-8:
                n = dx / dist
            else:
                n = np.array([1.0, 0.0])
            penetration = rad - dist
            # relative normal velocity
            vn = float((b.v - a.v) @ n)
            # spring towards separation, damping against approach
            fn = self.k * penetration - self.c * vn
            # only push outwards (block attractive force when separating fast)
            if fn < 0 and penetration < 1e-4:
                continue
            F = fn * n
            a.force -= F
            b.force += F

        # Walls as penalties
        for b in world.bodies:
            for n, depth in world.wall_penetration(b):
                if depth <= 0.0:
                    continue
                # relative normal velocity
                vn = float(b.v @ n)
                fn = self.k * depth - self.c * vn
                if fn < 0 and depth < 1e-4:
                    continue
                F = fn * n
                b.force += F
