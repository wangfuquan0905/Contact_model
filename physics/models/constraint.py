# -*- coding: utf-8 -*-
"""
Constraint-based collision (impulse):
- For overlapping pairs, compute a normal impulse that instantaneously
  corrects the normal component of the relative velocity according to
  restitution e.
- Then apply a small positional correction (Baumgarte) to remove residual
  penetration and avoid jitter.
This operates on velocities/positions directly (no forces).

Params
------
restitution : float or None
    If None, use world.meta['target_e'] (default ~0.9)
beta : float
    Baumgarte coefficient in [0, 1]; larger -> faster positional correction.
slop : float
    Penetration tolerance; below this we ignore correction to reduce jitter.
"""
import math
import numpy as np
from .base import CollisionModel

class ConstraintCollision(CollisionModel):
    name = "constraint"

    def __init__(self, restitution=None, beta: float = 0.2, slop: float = 1e-4):
        self.restitution = restitution
        self.beta = float(beta)
        self.slop = float(slop)

    def _unit_normal(self, dx):
        dist2 = float(dx @ dx)
        if dist2 <= 1e-12:
            return 0.0, np.array([1.0, 0.0])
        dist = math.sqrt(dist2)
        if dist > 1e-8:
            return dist, dx / dist
        else:
            return dist, np.array([1.0, 0.0])

    def apply(self, world, h: float):
        # --- pairwise disc collisions (normal impulses) ---
        e_world = float(world.meta.get('target_e', 0.9)) if self.restitution is None else float(self.restitution)

        for i, j, a, b in world.each_pairs():
            # Broad-phase: overlap check
            dx = b.x - a.x
            dist, n = self._unit_normal(dx)
            rad = a.r + b.r
            penetration = rad - dist
            if penetration <= 0.0:
                continue  # no contact

            # Relative velocity along normal
            rv = b.v - a.v
            vn = float(rv @ n)  # >0 separating, <0 approaching

            # Effective inverse mass along the normal
            inv_mass = a.inv_m + b.inv_m
            if inv_mass == 0.0:
                # both infinite mass -> skip
                continue

            # ---- Velocity-level resolution via impulse ----
            # Desired post-collision normal velocity = -e * (pre normal velocity) if approaching
            # Impulse magnitude j on the normal:
            # j = -(1+e)*vn / (sum inv_m) ; clamp to j>=0 to avoid attraction when separating
            j = 0.0
            if vn < 0.0:  # only if approaching
                j = -(1.0 + e_world) * vn / inv_mass
                if j < 0.0:
                    j = 0.0

            impulse = j * n
            # Apply equal/opposite impulse to velocities
            if a.m < math.inf:
                a.v -= impulse * a.inv_m
            if b.m < math.inf:
                b.v += impulse * b.inv_m

            # ---- Position-level correction (Baumgarte) ----
            # Push objects apart a bit to remove residual penetration and prevent sticking.
            corr_mag = max(penetration - self.slop, 0.0)
            if corr_mag > 0.0:
                # scale by beta and inverse masses (heavier moves less)
                correction = (self.beta * corr_mag / max(inv_mass, 1e-12)) * n
                if a.m < math.inf:
                    a.x -= correction * a.inv_m
                if b.m < math.inf:
                    b.x += correction * b.inv_m

        # --- wall contacts as impulses ---
        for b in world.bodies:
            # For walls, treat the wall as infinite mass plane; only b changes.
            for n, depth in world.wall_penetration(b):
                # normal velocity of b against the wall
                if depth <= 0.0:
                    continue
                vn = float(b.v @ n)
                if b.inv_m == 0.0:
                    continue

                # velocity-level impulse
                e_use = e_world
                j = 0.0
                if vn < 0.0:  # approaching wall
                    j = -(1.0 + e_use) * vn / (b.inv_m)
                    if j < 0.0:
                        j = 0.0
                impulse = j * n
                if b.m < math.inf:
                    b.v += impulse * b.inv_m

                # positional correction against wall (push inside the box)
                corr_mag = max(depth - self.slop, 0.0)
                if corr_mag > 0.0 and b.m < math.inf:
                    correction = (self.beta * corr_mag / b.inv_m) * n * b.inv_m
                    # 上面写法等价于直接: correction = self.beta * corr_mag * n
                    # 之所以保留形式，是为了和双体情形的分配保持一致
                    b.x += self.beta * corr_mag * n
