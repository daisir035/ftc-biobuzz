# Flower Cycle Implementation Plan

**Goal:** Implement the user's three-shot, two-collection sequence with early flywheel spin-up.
**Architecture:** One new autonomous file, existing subsystem commands, direct Ivy groups.
**Tech Stack:** Kotlin, Ivy 1.1.1, Pedro 3.0.1, FTC SDK 11.2.1.
**Spec:** ../specs/2026-11-01-flower-cycle-design.md

## Constraints

- Preserve current work and existing op-modes; no deployment.
- User draws paths; unconfigured routes refuse START.
- Robot lifecycle, Alliance poses, logged commands and monotonic waits follow AI-GUIDE.md.

## Review focus

- Path-progress reset, missed markers, and spin-up continuing into feeding.
- Parametric path end without physical arrival must not release balls.
- Cancellation before child start and during feeding must stop mechanisms.
- No encoder speed convergence must time out without feeding.
- Both collection cycles and all three shots must execute in order.

## Task 1: Routine and verification

- [x] Add FlowerCycleAutoTest with real Ivy/Pedro fixtures and simulated motor feedback.
- [x] Run the new tests before implementation and observe failure (new types absent).
- [x] Add FlowerCycleAuto.kt: RED/BLUE classes, editable path section, config, routine and lifecycle.
- [x] Verify sequence, early spin-up, feed gates, cancellation, repeated cycles and global timeout.
- [x] Update DEVELOPMENT.md and AI-GUIDE.md enabled-op-mode list.
- [x] Run :TeamCode:testDebugUnitTest and :TeamCode:assembleDebug; inspect results.
- [x] Review the final change independently and address material findings.

## Verification and review

2026-11-01: Eight new tests pass; full Kotlin suite passes 325 tests with zero
failures/errors; assembleDebug succeeds. No deployment or hardware operation.

Independent review found residual Pedro HOLD surviving cancellation after a
completed hold command. Reproduced with an off-target cancellation test (RED),
then added a guarded cleanup companion to stop the drive when the routine ends
or is cancelled (GREEN, full suite and assembly passed again).

Physical scoring effectiveness, servo calibration, user-drawn geometry and season
legality need field validation. No code-review findings were deferred.
