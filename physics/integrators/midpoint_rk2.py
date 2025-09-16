# -*- coding: utf-8 -*-
from .base import Integrator

class MidpointRK2(Integrator):
    name = "midpoint_rk2"

    def step(self, world, h: float):
        # 一次力评估的 RK2（中点法）版本，适配当前“力已预先累加”的框架
        for b in world.bodies:
            if not hasattr(b, "m") or b.m == float("inf"):
                continue
            a = b.force * b.inv_m          # k1_v
            v_half = b.v + 0.5 * h * a
            b.x += h * v_half              # x_{t+h} = x_t + h * v_half
            b.v += h * a                   # v_{t+h} = v_t + h * k1_v
