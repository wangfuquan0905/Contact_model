# -*- coding: utf-8 -*-
class Integrator:
    name = "base"
    def step(self, world, h: float):
        """Update states for one substep of size h.
        world.bodies[*].force 已经在这一子步开始时累加完毕（重力/外力/接触）。
        """
        raise NotImplementedError
