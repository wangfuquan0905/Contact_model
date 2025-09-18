# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from physics.world import World
from physics.bodies import Disc


class ControlState:
    def __init__(self):
        self.sel = 0      # selected body index
        self.fx = 0.0
        self.fy = 0.0
        self.F = 120.0
        self.keys = set()
        self.quit = False

    def on_key(self, event, world: World, press=True):
        key = event.key
        if press:
            self.keys.add(key)
        else:
            self.keys.discard(key)

        def held(k): return k in self.keys

        # movement forces
        self.fx = (held('right') - held('left')) * self.F
        self.fy = (held('up') - held('down')) * self.F

        # one-shot keys on press
        if press:
            if key in ('tab',):
                if len(world.bodies) > 0:
                    self.sel = (self.sel + 1) % len(world.bodies)
            if key == 'shift+tab':
                if len(world.bodies) > 0:
                    self.sel = (self.sel - 1) % len(world.bodies)
            if key == ' ':
                for b in world.bodies:
                    b.v[:] = 0.0
            if key in ('g',):
                world.gravity_on = not world.gravity_on
            if key in ('+', '='):
                r = 0.3
                m = np.pi * r * r
                world.add(Disc(
                    x=np.array([world.W*0.5, world.H*0.5], dtype=float),
                    v=np.array([0.0, 0.0], dtype=float),
                    r=r, m=m,
                    color=f"C{len(world.bodies)%10}"
                ))
            if key in ('-', '_') and len(world.bodies) > 0:
                world.bodies.pop()
                if len(world.bodies) > 0:
                    self.sel %= len(world.bodies)
                else:
                    self.sel = 0
            if key in ('m',):
                pass  # wired by caller
            if key in ('q', 'escape'):
                self.quit = True

        # update control forces map
        for i, _ in enumerate(world.bodies):
            world.control_forces[i] = np.array([0.0, 0.0])
        if len(world.bodies) > 0:
            world.control_forces[self.sel] = np.array([self.fx, self.fy])


def launch_viz(world: World, on_cycle_model=None, on_cycle_integrator=None):
    ctrl = ControlState()

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(0, world.W); ax.set_ylim(0, world.H); ax.set_aspect('equal', adjustable='box')
    ax.set_title("2D Discs — collision & integrator strategies")
    rect = plt.Rectangle((0,0), world.W, world.H, fill=False, lw=1.5)
    ax.add_patch(rect)

    patches = []
    for i, b in enumerate(world.bodies):
        c = plt.Circle((b.x[0], b.x[1]), b.r, ec=b.color, fc='none', lw=2.0)
        ax.add_patch(c)
        patches.append(c)

    # 文本显示
    info = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                   va='top', ha='left', fontsize=10)
    info.set_animated(True)

    def ensure_patches():
        nonlocal patches
        if len(patches) < len(world.bodies):
            for i in range(len(patches), len(world.bodies)):
                b = world.bodies[i]
                c = plt.Circle((b.x[0], b.x[1]), b.r, ec=b.color, fc='none', lw=2.0)
                ax.add_patch(c); patches.append(c)
        elif len(patches) > len(world.bodies):
            extra = patches[len(world.bodies):]
            for p in extra:
                p.remove()
            patches = patches[:len(world.bodies)]

    def update(_):
        if ctrl.quit:
            plt.close(fig)
            return patches + [info]

        notice = world.meta.pop('notice', None)
        world.step()
        ensure_patches()

        # 更新圆形位置/样式
        for i, (p, b) in enumerate(zip(patches, world.bodies)):
            p.center = (b.x[0], b.x[1])
            p.set_radius(b.r)
            if len(world.bodies) > 0 and i == (ctrl.sel % max(1, len(world.bodies))):
                p.set_edgecolor(b.color)
                p.set_linewidth(4.0)
            else:
                p.set_edgecolor(b.color)
                p.set_linewidth(2.0)

        # 选中刚体的速度数值
        vx = vy = vmag = 0.0
        if len(world.bodies) > 0:
            b_sel = world.bodies[ctrl.sel % len(world.bodies)]
            vx, vy = float(b_sel.v[0]), float(b_sel.v[1])
            vmag = float(np.linalg.norm(b_sel.v))

        info.set_text(
            f"model={world.meta.get('model_name','?')}  integrator={world.meta.get('integrator_name','?')}  "
            f"N={len(world.bodies)}  dt={world.dt:.4f}  g={'on' if world.gravity_on else 'off'}\n"
            f"Selected #{ctrl.sel}: V=({vx:+.3f}, {vy:+.3f}), |v|={vmag:.3f}\n"
            f"controls: arrows=force  TAB=next  m=cycle-model  i=cycle-integrator  +/-=add/rem  g=gravity  space=stop  r=reset\n"
            + (f"NOTICE: {notice}" if notice else "")
        )

        return patches + [info]

    ani = FuncAnimation(
        fig, update, blit=True,
        interval=int(world.dt*1000),
        cache_frame_data=False
    )

    def on_key_press(e):
        if e.key == 'm' and callable(on_cycle_model):
            on_cycle_model()
        if e.key == 'i' and callable(on_cycle_integrator):
            on_cycle_integrator()
        ctrl.on_key(e, world, press=True)

    def on_key_release(e):
        ctrl.on_key(e, world, press=False)

    fig.canvas.mpl_connect('key_press_event', on_key_press)
    fig.canvas.mpl_connect('key_release_event', on_key_release)
    plt.show()
