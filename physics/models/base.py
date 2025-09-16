# -*- coding: utf-8 -*-
class CollisionModel:
    name = "base"
    def apply(self, world, h: float):
        raise NotImplementedError
