# -*- coding: utf-8 -*-
from dataclasses import dataclass
import numpy as np
import math

Vec = np.ndarray

@dataclass
class Disc:
    x: Vec  # position (2,)
    v: Vec  # velocity (2,)
    r: float
    m: float
    color: str = "C0"
    force: Vec = None  # accumulated external force (2,)

    def __post_init__(self):
        if self.force is None:
            self.force = np.zeros(2, dtype=float)

    @property
    def inv_m(self) -> float:
        return 0.0 if self.m == math.inf else 1.0 / self.m
