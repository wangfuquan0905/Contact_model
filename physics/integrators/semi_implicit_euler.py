# -*- coding: utf-8 -*-
from .base import Integrator

class SemiImplicitEuler(Integrator):
    name = "semi_implicit_euler"

    def step(self, world, h: float):
        # v_{t+h} = v_t + (F/m) h
        # x_{t+h} = x_t + v_{t+h} h
        for b in world.bodies:
            if not hasattr(b, "m") or b.m == float("inf"):
                continue
            b.v += (h * b.force) * b.inv_m
            b.x += h * b.v
