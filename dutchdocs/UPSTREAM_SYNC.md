# Upstream sync record

## 2026-10-25: selective sync from `maxthegray/ftc-biobuzz`

### Revisions and method

- Local branch before sync: `master` at `dc9ea99` (`完成了底盘的表征`).
- Merge base: `7f5043b`.
- Upstream source: `upstream/master` at `89622ee` (`Simplify README`).
- Applied a squash merge without committing, then applied no-commit inverse
  patches for the exclusions below. The result is a working-tree changeset on
  `master`; no commit, push, deployment, or robot install was performed.

### Included upstream work

- MaxScope run comparison, field/3D views, traction and battery analysis, plus
  the supporting wheel, encoder, hub-current and hub-voltage logging.
- Automatic public subsystem state logging, centralized events, logging fault
  isolation, and the consolidated logging documentation.
- Localizer fault-policy cleanup, command/subsystem fault handling, drive
  updates, Panels field-view extraction, and telemetry cleanup.
- `VoltageCompensatedMecanum` and hub power sampling for follower path, hold,
  turn, and AutoTune power compensation.
- PS5-style gamepad API, current TeleOp controls, and match rumble alerts.
- Consolidated `Vision Test` diagnostics and vision configuration organization,
  while retaining the full vision implementation described below.
- `Just Path RED` and `Robot Centric Drive`; removal of the superseded
  `TwoTipAuto` and unused vision lab-record workflow.
- Other upstream documentation, tests, and non-robot-specific tool updates not
  covered by an exclusion.

### Deliberately excluded upstream commits

| Commit | Upstream change | Decision and reason |
|---|---|---|
| `2b0f404` | Simplified vision to alliance filtering and HIVE pivot aim | Reverted. Keep the ball camera, lens calibration, HIVE state/tracking, 3D goal geometry, held-goal projection, `PoseHistory`, settings-change logging, and their tests. The earlier consolidated `Vision Test` remains. |
| `7728b54` | Replaced `monotonicWaitMs` with Ivy `waitMs` | Reverted. Ivy 1.1.1 still uses `System.currentTimeMillis()`; autonomous delays and timeouts keep the monotonic clock. Non-actuator match rumble timing retains upstream's `waitMs`. |
| `d92e2d2` | Restored upstream CAD Pinpoint offsets | Reverted. They belong to a different chassis. |
| `ae5ef5e` | Adopted upstream tuned Pinpoint offsets | Reverted. Preserve this robot's measured offsets and directions. |
| `830cfdf` | Adopted upstream sensorbot Foresight tuning | Reverted. Preserve this robot's AutoTune coefficients. |

The complete old upstream vision remains available at upstream tag
`vision-full`, but this sync restores the pre-`2b0f404` implementation directly
rather than switching to that tag.

### Conflict and local-code resolutions

- `pedro/Constants.java`: kept local Pinpoint values
  (`x = 1.4569476270300197`, `y = -0.9436713241216704`), encoder directions,
  motor directions, and all local Foresight coefficients. Added the new
  battery-supplier `create` signature and compensated drivetrain.
- `NOMINAL_VOLTS` is `12.0`, the reference specified for future AutoTune runs,
  not upstream's robot-specific `9.5`. The preserved Foresight values predate
  voltage compensation and are **not considered validated with compensation**.
- `RobotConfig.kt`: kept local drive names `FL`, `FR`, `BL`, `BR`, all shooter
  names (`L`, `R`, `shoot`), and added upstream vision/match constants.
- `ShooterSubsystem.kt`: kept the complete dual/single flywheel implementation,
  PIDF handling, commands, shot model, and safe stop behavior. Its explicit
  logger preserves all previous channels, including `targetRpm = NaN` when
  stopped and the private applied-PIDF values.
- `DefaultAuto.kt`: kept local poses, RED/BLUE behavior, start delay, and
  monotonic timeouts. Adapted battery input, explicit Pinpoint status-fault
  policy, and PS5 dpad API.
- `JustPathAuto.kt` and `ExampleAuto.kt`: use `monotonicWaitMs` for autonomous
  delays/timeouts. Shooter diagnostics use R2 and retain their hardware preflight.
- `AI-GUIDE.md`, `DEVELOPMENT.md`, and `OPERATIONS.md`: reconciled with the actual
  hardware names, enabled op-modes, full vision, automatic logging, and tuning
  state rather than accepting either side wholesale.

### Host verification

`make` is not installed on this Windows host, so the Makefile's underlying
commands were run directly with Android Studio's JDK 21:

- `./gradlew :TeamCode:testDebugUnitTest` — **passed, 291 tests**. The first run
  exposed the upstream test's shooter-skeleton assumption; the test now checks
  the retained implementation's seven explicit log channels.
- `./gradlew :TeamCode:assembleDebug` — **passed** on the final changeset.
- `node --test tools/maxscope/viewer/core.test.mjs` — **passed, 37 tests**.
- `python3 -m unittest tools/logs/test_analyze_wpilog.py
  tools/maxscope/test_log_viewer.py tools/maxscope/test_traction.py
  tools/maxscope/test_battery.py` — **45 of 46 tests passed**. The remaining
  `test_arbitrary_file_paths_cannot_be_requested` could not set up its symlink:
  Windows returned `WinError 1314` (privilege not held) before any assertion.
  WSL is not installed. This security case still needs a run on a host where
  unprivileged symlinks are available.
- `git diff --check HEAD -- . ':(exclude)tools/maxscope/viewer/vendor/**'` —
  **passed**. The upstream-vendored Three.js source is excluded because it
  carries its own whitespace; no conflict markers or unresolved merge/revert
  state remain.

### Required on-robot follow-up

Host tests cannot validate these changes physically. Before competition use:

1. Re-run Foresight AutoTune through voltage compensation with
   `NOMINAL_VOLTS = 12.0`, paste the resulting values, and repeat Hold/Line/Curve
   tests on more than one battery charge level.
2. Check `Drive/voltageScale`, hub voltage/current, and path behavior in MaxScope.
3. Run the existing vision validation pages for the ball camera and HIVE tracker;
   the full algorithms were retained, not revalidated by this sync.
4. Verify R2 shooter tests, PIDF logging, safe stop, and both Default Auto
   alliances with the robot initially on blocks and then in a clear field area.

## 2026-11-01: record upstream `89622ee` in history

- Merged `sync-upstream` (a real `git merge` of `upstream/master` `89622ee`
  plus small fixes) into `auto-framework` with this branch's tree kept
  unchanged. Upstream had already been applied selectively on 2026-10-25
  (above); this merge only records that, so the next sync starts from
  `89622ee` instead of re-presenting the same 60+ commits as conflicts.
- Every conflict and every cleanly merged upstream change resolved to this
  branch: the exclusions above still hold (full vision, `PoseHistory`,
  `monotonicWaitMs`, `registerAfter`, `toggleOnTrue`, this robot's Pinpoint
  and Foresight values). The `sync-upstream` fixes are superseded here by
  `Scoring`, the Shooter NaN guard and the Feeder `spinning` gate.
- Next sync: `git merge upstream/master` and resolve only what upstream
  changed after `89622ee`.

### AutoTune procedures vs Pedro-Pathing/Quickstart

- `PinpointTuner.java` is the team's fix 3494d7a ("Read Pinpoint direction
  before Stop, not after"). It was merged into Pedro-Pathing/Quickstart
  (PR #115, 2026-10-08), and the file here is byte-identical to the
  official Quickstart `HEAD` (checked 2026-11-01). It is official code now.
- `MecanumTuner.java` matches both b4312385 and `HEAD`.
- `ForesightTuner.java` and `Tests.java` are still b4312385. Quickstart has
  since changed them (Foresight NaN/runaway stop, braking-test stop
  condition, Distance input in Line/Curve tests) alongside Pedro 3.0.2 and
  AutoTune 1.0.3. They are not copied: AutoTune 1.0.3 needs Sloth 0.3.2
  (see AI-GUIDE). Re-copy all four together when the toolchain moves.

## 2026-11-01: review of the 2026-10-25 exclusions

Re-checked independently instead of trusting the record above:

- `7728b54` (`monotonicWaitMs` → Ivy `waitMs`): **still excluded.** Ivy
  1.1.1's `Commands.waitMs` uses `System.currentTimeMillis()` (checked in the
  published `core-1.1.1-sources.jar`).
- `d92e2d2`, `ae5ef5e`, `830cfdf` (sensorbot Pinpoint/Foresight values):
  **still excluded**; they are another chassis.
- `2b0f404` (simplified vision): **now taken.** This robot has only a
  Limelight, used for AprilTags; no ball camera, and no turret yet for the
  HIVE tracker. Vision is upstream's `GoalAim` + `LimelightSubsystem` +
  `Vision Test`; `PoseHistory`, `registerAfter` and `SettingsChangeLog` went
  with it. The full estimator is at upstream tag `vision-full`.
- Drive motor names are back in `RobotConfig.Drive` (left front, right front,
  left rear, right rear) and `Constants` points at them, as upstream does.
