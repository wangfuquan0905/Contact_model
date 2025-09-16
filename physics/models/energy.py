# -*- coding: utf-8 -*-
"""Energy-based collision — scaffold.
Not implemented in this version.
A future version will push bodies along gradients of a smooth contact energy.
"""
import math
import numpy as np
from .base import CollisionModel

class EnergyCollision(CollisionModel):
    name = "energy"
    def __init__(self,mu = 0.5, eps = 1e-4, d0 = 1e-4):
        self.mu = float(mu)
        self.eps = float(eps)
        self.d0 = float(d0)
        
    def apply(self, world, h: float):
        for i, j, a, b in world.each_pairs():
            dx = b.x - a.x
            dist2 = float(dx @ dx)
            rad = a.r + b.r
            actdis = rad + self.d0
            if dist2 > actdis*actdis:
                continue
            else:
                dist = math.sqrt(dist2) if dist2 > 1e-12 else 0.0
                n = dx / dist if dist > 1e-8 else np.array([1.0, 0.0])
                penetration = dist - rad
                #p = max(penetration, 0.0)
                #Psi = (self.mu/2)* ((self.d0 - penetration)**2)/(p + self.eps)
                if penetration > 0:
                    dpsi = (self.mu/2)*((penetration - self.d0)*(penetration + self.d0 + 2*self.eps))/((penetration + self.eps)**2)
                else:
                    dpsi = self.mu*(penetration - self.d0)/(self.eps)
                F = dpsi*n*(-1)
                a.force -= F
                b.force += F
        
        for b in world.bodies:
            for n, depth in world.wall_penetration(b):
                penetration = -float(depth)
                if penetration - self.d0 <= 0:
                    #p = max(penetration, 0.0)
                    #Psi = (self.mu/2)* ((self.d0 - penetration)**2)/(p + self.eps)
                    if penetration > 0:
                        dpsi = (self.mu/2)*((penetration - self.d0)*(penetration + self.d0 + 2*self.eps))/((penetration + self.eps)**2)
                    else:
                        dpsi = self.mu*(penetration - self.d0)/(self.eps)
                    F = dpsi*n*(-1)
                    b.force += F
        return
