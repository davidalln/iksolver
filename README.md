15-664 MiniProject 1
====================

Author: David Allen (dallen1)

An inverse kinematics solver. There are two modes in the program: drawing the skeleton and moving the target for the end effector. Here a skeleton is defined as having a fixed root and a single end effector.

Controls
--------
* ESC to quit program

### Drawing Mode
* RMB to draw joints (first click to set the root, then click again to build joints)
* SPACE to switch to IK mode (end effector switches from square to circle)

### IK Mode
* RMB to move IK target
* SPACE to switch to drawing mode (deletes the current skeleton)

Code Layout
-----------
* **main.cpp** - main rendering and input routines
* **ikskel.cpp** - inverse kinematics routines for the skeleton

Implemented IK Solvers
----------------------
* CCD

