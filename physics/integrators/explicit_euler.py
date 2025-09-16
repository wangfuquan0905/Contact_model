# -*- coding: utf-8 -*-
from .base import Integrator

class ExplicitEuler(Integrator):
    name = "explicit_euler"

    def step(self, world, h: float):
        # x_{t+h} = x_t + v_t h
        # v_{t+h} = v_t + (F/m) h
        for b in world.bodies:
            if not hasattr(b, "m") or b.m == float("inf"):
                continue
            b.x += h * b.v
            b.v += (h * b.force) * b.inv_m
