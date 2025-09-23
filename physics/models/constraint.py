# -*- coding: utf-8 -*-
"""
LCP-based collision with friction (Stewart–Trinkle, Linearized Friction Cone).
- Impulse–velocity time stepping
- Linearized friction cone with nd (even) rays D_i, nonnegative edge variables
- Per-contact slack scalar γ to unify stick/slide
- Solved by Projected Gauss–Seidel (PGS), warm-started by zeros
- Optional Baumgarte position stabilization

This file is shape-agnostic for 'solve' phase. You only need the contact list:
    contacts = [
        {
          "a": body_a, "b": body_b,
          "n": unit_normal_world(np.ndarray shape (2,)),   # pointing from a -> b
          "depth": positive_penetration_depth_float,       # >0 means interpenetration
          "point": world_point_of_contact (optional)       # for rotation torque later
        },
        ...
    ]
For walls, set a=None or b=None; provide the existing body on side "b" and n pointing
from virtual wall -> body.

Rotations:
- If you later add rotation, extend J assembly (linear + angular blocks), and A = J M^{-1} J^T
  will remain valid without touching the LCP logic.

References to formulation: Wei-Cheng Huang tutorial slides (2025/2/19), “From NCP to LCP (S–T)”
"""

import math
import numpy as np
from .base import CollisionModel


def _unit(v):
    n = float(v @ v)
    if n <= 1e-30:
        return np.array([1.0, 0.0])
    return v / math.sqrt(n)


class ConstraintCollision(CollisionModel):
    name = "lcp_st"

    def __init__(self,
                 nd: int = 2,           # number of friction rays (even, >=4)
                 mu_default: float = 0.3,
                 restitution: float = 0.9,   # use De Saxcé style e via b-term; here simple v* is used
                 beta: float = 0.2,     # Baumgarte positional stabilization
                 slop: float = 1e-4,
                 max_iters: int = 200,
                 tol: float = 1e-6):
        assert nd == 2
        self.nd = int(nd)
        self.mu_default = float(mu_default)
        self.restitution = float(restitution)
        self.beta = float(beta)
        self.slop = float(slop)
        self.max_iters = int(max_iters)
        self.tol = float(tol)

    # ------------------------------
    # Contact generation (shape-agnostic API expected from world)
    # ------------------------------
    def _gather_contacts(self, world):
        contacts = []

        # pairwise: discs now; general shapes -> replace with your broad/narrow phase
        for i, j, a, b in world.each_pairs():
            dx = b.x - a.x
            dist2 = float(dx @ dx)
            rad = getattr(a, "r", 0.0) + getattr(b, "r", 0.0)
            if dist2 <= (rad * rad):
                dist = math.sqrt(dist2) if dist2 > 1e-16 else 0.0
                n = _unit(dx if dist > 1e-12 else np.array([1.0, 0.0]))
                depth = rad - dist
                if depth > 0.0:
                    contacts.append({"a": a, "b": b, "n": n, "depth": depth, "mu": self._mu_of(a, b)})

        # walls
        for b in world.bodies:
            for n, depth in world.wall_penetration(b):
                if depth > 0.0:
                    contacts.append({"a": None, "b": b, "n": n, "depth": depth, "mu": getattr(b, "mu", self.mu_default)})

        return contacts

    def _mu_of(self, a, b):
        mu_a = getattr(a, "mu", self.mu_default) if a is not None else self.mu_default
        mu_b = getattr(b, "mu", self.mu_default) if b is not None else self.mu_default
        return 0.5 * (mu_a + mu_b)

    # ------------------------------
    # Assemble LCP blocks for S–T with LFC + slack γ
    # Variables per contact c:
    #   z_c = [λ_N; λ_{D1}..λ_{Dnd}; γ]  (all >=0)
    # Residuals (complementary):
    #   w_N  = J_N v_{k+1} - v_N*                  ⟂ λ_N
    #   w_Ti = (e_i γ) + (J_Ti v_{k+1})            ⟂ λ_{Di}
    #   w_γ  = μ λ_N - e^T λ_T                     ⟂ γ
    #
    # where v_{k+1} = v_free + M^{-1} J^T λ_c  and A = J M^{-1} J^T
    # ------------------------------
    def apply(self, world, h: float):
        contacts = self._gather_contacts(world)
        if not contacts:
            return

        # --- helpers ---
        def invM_of(body):
            return 0.0 if (body is None or body.m == math.inf) else body.inv_m

        # --- 2D LFC: use exactly two rays [t, -t] ---
        lfc_dirs = []
        for c in contacts:
            n = c["n"]
            t = _unit(np.array([-n[1], n[0]], dtype=float))
            lfc_dirs.append(np.vstack([t, -t]))   # shape (2,2)
        ND = 2
        dim_c = 1 + ND + 1   # [λN, λT1, λT2, γ]

        # --- in-place PGS on velocities ---
        lambdas = np.zeros(dim_c * len(contacts))

        for it in range(self.max_iters):
            max_delta = 0.0
            off = 0
            for ci, c in enumerate(contacts):
                a, b, n = c["a"], c["b"], c["n"]
                mu, depth = c.get("mu", self.mu_default), c["depth"]
                D = lfc_dirs[ci]      # (2,2)

                va = np.zeros(2) if a is None else a.v
                vb = np.zeros(2) if b is None else b.v
                vrel = vb - va
                vN = float(vrel @ n)
                vTi = D @ vrel

                # ---- reference normal velocity (restitution + strong push-out if penetrating) ----
                vN_star = 0.0
                if vN < 0.0:
                    vN_star = - self.restitution * vN
                if depth > self.slop:
                    # 强制给一个“愿望分离速度”，即使 vN >= 0 也推（避免贴墙抖动时吸进去）
                    vN_star += self.beta * (depth - self.slop) / max(h, 1e-6)

                inv_ma = invM_of(a); inv_mb = invM_of(b)
                A_nn = inv_ma + inv_mb
                if A_nn <= 0.0:
                    off += dim_c
                    continue

                lamN = lambdas[off + 0]
                lamT = lambdas[off + 1: off + 3]
                gamma = lambdas[off + 3]

                # --- λN ---
                wN = vN - vN_star
                new_lamN = max(0.0, lamN - wN / A_nn)
                dlamN = new_lamN - lamN
                if dlamN != 0.0:
                    imp = dlamN * n
                    if a is not None and a.m < math.inf: a.v -= imp * a.inv_m
                    if b is not None and b.m < math.inf: b.v += imp * b.inv_m
                    lamN = new_lamN
                    max_delta = max(max_delta, abs(dlamN))

                # refresh vrel for tangential update
                va = np.zeros(2) if a is None else a.v
                vb = np.zeros(2) if b is None else b.v
                vrel = vb - va
                vTi = D @ vrel


                gamma_relax = 0.2   # 你现在用的 0.2
                omega_T = 1.2       # λ_T 的 SOR 超松弛
                eps_t = 1e-6 * (inv_ma + inv_mb + 1.0) / max(h, 1e-6)
                A_tt = A_nn + eps_t
                # 让 γ 与 λ_T 在同一外层迭代里先小收敛一下
                for _ in range(2):  # 1~3 次足够
                    # γ：欠松弛投影
                    w_gamma = mu * lamN - (lamT[0] + lamT[1])
                    gamma = max(0.0, gamma - gamma_relax * w_gamma)

                    # 速度刷新（因为上一步可能改了 λ_T/γ→速度）
                    va = np.zeros(2) if a is None else a.v
                    vb = np.zeros(2) if b is None else b.v
                    vrel = vb - va
                    vTi = D @ vrel

                    # λT：PGS + SOR
                    for i in (0, 1):
                        wTi = vTi[i] + gamma
                        new_lamTi = max(0.0, lamT[i] - omega_T * wTi / A_tt)   # PSOR
                        
                        dlam = new_lamTi - lamT[i]
                        if dlam != 0.0:
                            imp = dlam * D[i]
                            if a is not None and a.m < math.inf: a.v -= imp * a.inv_m
                            if b is not None and b.m < math.inf: b.v += imp * b.inv_m
                            lamT[i] = new_lamTi
                            max_delta = max(max_delta, abs(dlam))

                # write back
                lambdas[off + 0] = lamN
                lambdas[off + 1: off + 3] = lamT
                lambdas[off + 3] = gamma
                off += dim_c
            
            # --- convergence check ---
            max_residual = 0.0
            for ci, c in enumerate(contacts):
                off = ci * dim_c
                lamN = lambdas[off + 0]
                lamT = lambdas[off + 1: off + 3]
                gamma = lambdas[off + 3]
                D = lfc_dirs[ci]                      
                mu = c.get("mu", self.mu_default)      
                depth = c["depth"]   
                
                # 当前相对速度
                a, b, n = c["a"], c["b"], c["n"]
                va = np.zeros(2) if a is None else a.v
                vb = np.zeros(2) if b is None else b.v
                vrel = vb - va

                vN = float(vrel @ n)
                vTi = D @ vrel   # D 是切向基
                
                vN_star = 0.0
                if vN < 0.0:
                    vN_star = - self.restitution * vN
                if depth > self.slop:
                    vN_star += self.beta * (depth - self.slop) / max(h, 1e-6)
                wN = vN - vN_star
                wT = vTi + gamma
                w_gamma = mu * lamN - (lamT[0] + lamT[1])
                
                # 计算违反度
                max_residual = max(max_residual,
                                abs(min(lamN, wN)),
                                abs(min(lamT[0], wT[0])),
                                abs(min(lamT[1], wT[1])),
                                abs(min(gamma, w_gamma)))

            if max_delta < self.tol and max_residual < 1e-4: break
