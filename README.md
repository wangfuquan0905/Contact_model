Download
Run main.py
"""
Two Rigid Balls (N discs) — switchable collision models
Usage:
    python main.py --model penalty --num 2 --dt 0.008 --substeps 1 --e 0.9

Controls:
    Arrow keys  : apply force to selected ball (hold)
    TAB         : select next ball
    SHIFT+TAB   : select previous ball
    m           : cycle collision model (penalty -> constraint -> energy)
    + / -       : add / remove a ball
    g           : toggle gravity
    space       : zero velocities of all balls
    r           : reset random initial state
    q / ESC     : quit

Note:
 - Only the 'penalty' model is implemented in this version.
 - 'constraint' and 'energy' models are scaffolded and will print a notice if selected.
"""
