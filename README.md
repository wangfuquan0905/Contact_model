Two Rigid Balls (N Discs) — Switchable Collision Models
=======================================================

This project is a minimal 2D rigid-disc simulator with support for multiple
collision handling models. Currently only the penalty model is implemented;
other models are scaffolded.

-------------------------------------------------------
Usage
-------------------------------------------------------

Run the main script:

    python main.py

Command-line Arguments:
    --model     : collision model to use (penalty, constraint, energy)
    --num       : number of discs
    --dt        : time step (seconds)
    --substeps  : number of sub-steps per frame
    --e         : restitution coefficient (0–1)

-------------------------------------------------------
Controls (Keyboard)
-------------------------------------------------------

    Arrow keys   : Apply force to selected ball (hold to apply continuously)
    TAB          : Select next ball
    SHIFT+TAB    : Select previous ball
    m            : Cycle collision model (penalty -> constraint -> energy)
    + / -        : Add / remove a ball
    g            : Toggle gravity
    SPACE        : Zero velocities of all balls
    r            : Reset to a random initial state
    q / ESC      : Quit simulation

-------------------------------------------------------
Notes
-------------------------------------------------------

- all 3 model is fully implemented in this version.

-------------------------------------------------------

Roadmap
-------------------------------------------------------

- [x] Penalty-based collision model
- [x] Constraint-based collision model (impulse/LCP)
- [x] Energy-based collision model (potential methods)
