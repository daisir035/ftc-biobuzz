# Flower cycle autonomous

Sequence: timed preload shot, lower shovels, collect at the first FLOWER,
cross to the far shooting point and shoot, collect at the second FLOWER,
return home and shoot. The user supplies all four paths and field poses.

Use existing Ivy groups and subsystem command factories. Keep teleop and
existing autos unchanged. New Flower Cycle RED/BLUE op-modes share one routine.
The RED path section starts unconfigured and refuses START until filled in.
BLUE uses Alliance.poses(), with the same sequence.

Each shot has an adjustable feed duration. Initial/home shots use nearRpm;
the far shot uses farRpm. A configurable path-progress marker (initially 0.7)
starts the flywheel on each shooting approach; spin-up continues through
arrival and feeding. If the marker is missed, spin-up starts at approach end.
Feed only after measured pose and RPM are ready; skip a timed-out shot.
Collection runs intake plus low-power feeder assistance with adjustable dwell.
Lowering gets an adjustable servo settling wait; shovels remain down afterward.

Use per-path timeouts and a 29-second routine limit. Localizer pose loss cancels
the routine; status-only faults keep running. Commands and steps are logged.
No deployment or hardware motion is part of this change.

Alternative considered: a custom autonomous state machine. Existing Ivy
composition already handles requirements and cancellation, so use it directly.

## Supplied reference route

The user subsequently provided four curves. They replace the initial placeholders
and enable the route gate. RED poses: start (56, 8, -90°), first FLOWER
(9.7698, 46.05, 180°), far shot (61.0083, 133.8026, 90°), second FLOWER
(46.7669, 131.9023, 90°), return shot (61.3271, 133.0142, 90°).
All supplied control points and linear heading interpolation are preserved;
every endpoint and control point goes through Alliance.poses() for BLUE.
The return shot is adjacent to the far shot, so both now use farRpm; only the
initial preload shot uses nearRpm.
