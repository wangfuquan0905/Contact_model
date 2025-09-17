# -*- coding: utf-8 -*-
import argparse
import numpy as np
import random

from physics.world import World, rand_nonoverlap_positions
from physics.bodies import Disc
from physics.models.penalty import PenaltyCollision
from physics.models.constraint import ConstraintCollision
from physics.models.energy import EnergyCollision

from physics.integrators.semi_implicit_euler import SemiImplicitEuler
from physics.integrators.explicit_euler import ExplicitEuler
from physics.integrators.midpoint_rk2 import MidpointRK2

from rendering.viz import launch_viz

def make_world(args):
    W, H = args.box
    world = World(size=(W, H), dt=args.dt, substeps=args.substeps, gravity=args.gravity)
    # Create discs
    rng = np.random.default_rng(args.seed)
    radii = [args.r_min + (args.r_max-args.r_min)*rng.random() for _ in range(args.num)]

    # 用于位置
    centers = rand_nonoverlap_positions(
        n=args.num,
        radii=radii,
        box=(W, H),
        max_tries=2000,
        rng=rng
    )

    # 用于创建 Disc
    discs = []
    for i, ((x, y), r) in enumerate(zip(centers, radii)):
        m = np.pi * r * r
        v = rng.uniform(-1.0, 1.0, size=2)
        discs.append(Disc(x=np.array([x, y], dtype=float),
                        v=v.astype(float),
                        r=float(r),
                        m=float(m),
                        color=f"C{i%10}"))

    for d in discs:
        world.add(d)
    set_model(world, args)
    set_integrator(world, args)
    
    world.meta['target_e'] = args.e
    world.meta['k'] = args.k
    world.meta['c'] = args.c
    return world

def set_model(world, args_or_name):
    name = args_or_name.model if hasattr(args_or_name, 'model') else args_or_name
    name = name.lower()
    if name == 'penalty':
        world.set_collision_model(PenaltyCollision(k=world.meta.get('k', 800.0),
                                                   c=world.meta.get('c', 8.0),
                                                   e_target=world.meta.get('target_e', 0.9)))
    elif name == 'constraint':
        world.set_collision_model(ConstraintCollision())
    elif name == 'energy':
        world.set_collision_model(EnergyCollision())
    else:
        raise ValueError(f"Unknown model: {name}")
    world.meta['model_name'] = name
    return name

def set_integrator(world, args_or_name):
    name = args_or_name.integrator if hasattr(args_or_name, 'integrator') else args_or_name
    name = name.lower()
    if name in ('sie', 'semi', 'semi_implicit', 'symplectic'):
        world.set_integrator(SemiImplicitEuler()); name = 'sie'
    elif name in ('ee', 'explicit'):
        world.set_integrator(ExplicitEuler()); name = 'ee'
    elif name in ('rk2', 'midpoint'):
        world.set_integrator(MidpointRK2()); name = 'rk2'
    else:
        raise ValueError(f'Unknown integrator: {name}')
    world.meta['integrator_name'] = name
    return name

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--model', type=str, default='penalty', choices=['penalty', 'constraint', 'energy'])
    p.add_argument('--integrator', type=str, default='sie', choices=['sie', 'ee', 'rk2'])
    p.add_argument('--num', type=int, default=2, help='number of discs')
    p.add_argument('--box', type=float, nargs=2, default=[8.0, 5.0], help='box size W H')
    p.add_argument('--dt', type=float, default=0.008)
    p.add_argument('--substeps', type=int, default=1)
    p.add_argument('--e', type=float, default=0.9, help='target restitution (used by penalty for damping hint)')
    p.add_argument('--k', type=float, default=800.0, help='penalty stiffness')
    p.add_argument('--c', type=float, default=8.0, help='penalty damping')
    p.add_argument('--r_min', type=float, default=0.25)
    p.add_argument('--r_max', type=float, default=0.45)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--gravity', action='store_true', help='turn on gravity')
    return p.parse_args()

def main():
    args = parse_args()
    world = make_world(args)
    
    # 循环碰撞模型
    def cycle_model():
        order = {'penalty':'constraint','constraint':'energy','energy':'penalty'}
        set_model(world, order[world.meta['model_name']])

    # 循环积分器
    def cycle_integrator():
        order = {'sie': 'ee', 'ee': 'rk2', 'rk2': 'sie'}
        set_integrator(world, order[world.meta.get('integrator_name', 'sie')])

    # 传入两个回调
    launch_viz(world, on_cycle_model=cycle_model, on_cycle_integrator=cycle_integrator)

if __name__ == '__main__':
    main()
