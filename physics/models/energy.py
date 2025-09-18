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
    def __init__(self,mu = 0.5, eps = 1e-4, d0 = 1e-4, e = 0.3):
        self.mu = float(mu)
        self.eps = float(eps)
        self.d0 = float(d0)
        self.e = float(e)
    
    def _fn_from_reladis(self, reladis: float) -> float:# gradient of contact energy
        #p = max(reladis, 0.0)
        #Psi = (self.mu/2)* ((self.d0 - reladis)**2)/(p + self.eps)
        #-----gradient of Psi------
        if reladis > 0: 
            dpsi = (self.mu/2)*((reladis - self.d0)*(reladis + self.d0 + 2*self.eps))/((reladis + self.eps)**2)
        else:
            dpsi = self.mu*(reladis - self.d0)/(self.eps)  
              
        return max(0, -dpsi)
    
    def _knumerical(self,reladis: float, get_force_scalar) -> float:# numerical stiffness
        delta = max(1e-6, 1e-3 * self.d0)
        f_plus  = get_force_scalar(reladis + delta)
        f_minus = get_force_scalar(reladis - delta)
        k = (f_minus - f_plus) / (2.0 * delta)
        return max(k, 0.0)  # 数值安全
    
    def _zeta_from_e(self, e: float) -> float: # damping ratio from restitution
        e = max(1e-6, min(0.999999, float(e)))
        L = math.log(1.0 / e)
        return L / math.sqrt(math.pi**2 + L*L)
                                        
    def _cn(self, meff: float, e: float, reladis: float) -> float:# critical damping coefficient
        zeta = self._zeta_from_e(e)
        knum = self._knumerical(reladis, self._fn_from_reladis)
        #rint(f"meff: {meff}, knum: {knum}, zeta: {zeta}")
        return 2.0 * zeta * math.sqrt(meff * knum)
    
    def _proximity_weight(self, reladis: float):# [0, d0] -> [1, 0]
        t = (self.d0 - reladis) / self.d0
        if t <= 0:
            return 0.0
        elif t >= 1:
            return 1.0
        else:
            return t*t*(3 - 2*t)  # smoothstep
        
    def _meff_pair(self, a, b) -> float:# effective mass of a pair
        if a.inv_m + b.inv_m > 0.0:
            return 1.0 / (a.inv_m + b.inv_m)
        else:
            return math.inf
        
        
    def apply(self, world, h: float):
        for i, j, a, b in world.each_pairs():
            dx = b.x - a.x
            dist2 = float(dx @ dx)
            rad = a.r + b.r
            actdis = rad + self.d0
            #collision detection
            if dist2 > actdis*actdis: 
                continue
            else:
                dist = math.sqrt(dist2) if dist2 > 1e-12 else 0.0
                n = dx / dist if dist > 1e-8 else np.array([1.0, 0.0])
                reladis = dist - rad
                fn = self._fn_from_reladis(reladis)# gradient of contact energy
                F = fn*n
                a.force -= F
                b.force += F
                
                if reladis < self.d0: # in contact(dis < d0)
                    # relative normal velocity
                    vn = float((b.v - a.v) @ n)
                    if vn < 0.0: # only if approaching
                        meff = self._meff_pair(a, b)
                        cn = self._cn(meff, self.e, reladis)
                        w = self._proximity_weight(reladis)
                        Fdamp = (-cn * vn * w)*n
                        a.force -= Fdamp
                        b.force += Fdamp
        
        for b in world.bodies:
            for n, depth in world.wall_penetration(b):
                reladis = -float(depth)
                if reladis - self.d0 <= 0:
                    fn = self._fn_from_reladis(reladis)
                    F = fn*n
                    b.force += F

                    # relative normal velocity
                    vn = float(b.v @ n)
                    if vn < 0.0: # only if approaching
                        meff = b.m 
                        cn = self._cn(meff, self.e, reladis)
                        w = self._proximity_weight(reladis)
                        Fdamp = (-cn * vn * w)*n
                        b.force += Fdamp 
        return
