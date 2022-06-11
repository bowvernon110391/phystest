# Bowie's vehicle physics simulation (2D)

This program attempts to simulate vehicle physics using constrained dynamics.

## Features:

- constraint based simulation
- inherently stable (although not quite because we can't inject code into the physics lib)
- raycast physics, good for high velocity vehicle
- many configuration supported (FWD, RWD, 4WD with locked spool, AWD with open differential, Limited Slip Differential)
- all of those are modeled as constraints, which is plugged in into the solver

## Requires:

- pygame
- Box2D (the physics library)

## TODO:

- [ ] pacejka magic formula
- [ ] low speed method using relaxation, with slope gradient derrived from the pacejka formula