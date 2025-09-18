# -*- coding: utf-8 -*-
import numpy as np
import math
from typing import List, Tuple, Optional
from .bodies import Disc

class World:
    def __init__(self, size=(8.0, 5.0), dt=1/120, substeps:int=4, gravity:bool=False):
        self.W, self.H = size
        self.dt = float(dt)
        self.substeps = max(1, int(substeps))
        self.gravity_on = bool(gravity)
        self.bodies: List[Disc] = []
        self.model = None  # strategy object for collisions
        self.integrator = None  #
        self.meta = {}     # misc configuration (k, c, e, model name)
        
        # controls (forces applied from UI)
        self.control_forces = {}  # id -> np.array([fx,fy])

    def set_collision_model(self, model):
        self.model = model
        
    def set_integrator(self, integrator):
        self.integrator = integrator

    def add(self, d: Disc):
        self.bodies.append(d)

    def clear_forces(self):
        for b in self.bodies:
            b.force[:] = 0.0

    def step(self):
        h = self.dt / self.substeps
        for _ in range(self.substeps):
            self.clear_forces()
            # gravity
            if self.gravity_on:
                for b in self.bodies:
                    if b.m < math.inf:
                        b.force[1] -= 9.81 * b.m
            # user control forces
            for i, b in enumerate(self.bodies):
                f = self.control_forces.get(i)
                if f is not None:
                    b.force += f

            # model contributes contact forces (penalty), or impulses via velocity changes.
            if self.model is not None:
                self.model.apply(self, h)

            # 使用可插拔积分器
            if self.integrator is not None:
                self.integrator.step(self, h)
            else:
                # 若未设置，维持当前行为为半隐式欧拉（可选备份）
                for b in self.bodies:
                    if b.m < math.inf:
                        b.v += (h * b.force) * b.inv_m
                        b.x += h * b.v


    # --- helpers for model implementations ---
    def each_pairs(self):
        n = len(self.bodies)
        for i in range(n):
            for j in range(i+1, n):
                yield i, j, self.bodies[i], self.bodies[j]

    def wall_penetration(self, b: Disc):
        # return list of (normal, depth, contact_point approx)
        out = []
        # left wall x=0
        depth = b.r - b.x[0]
        out.append((np.array([1.0, 0.0]), depth))
        # right wall x=W
        depth = b.x[0] + b.r - self.W
        out.append((np.array([-1.0, 0.0]), depth))
        # bottom y=0
        depth = b.r - b.x[1]
        out.append((np.array([0.0, 1.0]), depth))
        # top y=H
        depth = b.x[1] + b.r - self.H
        out.append((np.array([0.0, -1.0]), depth))
        return out


def rand_nonoverlap_positions(n:int, radii, box, max_tries=2000, rng=None):
    if rng is None:
        rng = np.random.default_rng(0)
    W, H = box
    centers = []
    for i in range(n):
        r = radii[i]
        ok = False
        for _ in range(max_tries):
            x = float(r + (W - 2*r) * rng.random())
            y = float(r + (H - 2*r) * rng.random())
            ok = True
            for (cx, cy), rr in zip(centers, radii[:len(centers)]):
                dx, dy = x - cx, y - cy
                if dx*dx + dy*dy < (r+rr)**2 * 1.05:
                    ok = False
                    break
            if ok:
                centers.append((x, y))
                break
        if not ok:
            # fall back: place near edge, may overlap slightly
            centers.append((r + 0.5*i, r + 0.5*i))
    return centers
