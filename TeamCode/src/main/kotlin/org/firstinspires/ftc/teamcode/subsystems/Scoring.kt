package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs

/**
 * The ball path as one unit: intake → feeder → flywheel. Teleop and autos
 * build every collect and shot from these, so both behave the same.
 *
 * Not a subsystem: it owns no hardware, and each command requires the
 * subsystems it drives, so a shot takes the intake and feeder away from a
 * collect (newest wins).
 */
class Scoring(
    private val intake: IntakeSubsystem,
    private val feeder: FeederSubsystem,
    private val shooter: ShooterSubsystem,
    private val clock: Clock = Clock.SYSTEM,
    private val recordEvent: (String) -> Unit = {},
) {

    /**
     * While `active()` is true the intake collects and the feeder turns at
     * [FeederConfig.intakePower] so balls move up. When it goes false the
     * intake stops at once and the feeder waits
     * [FeederConfig.intakeReleaseDelayMs], reverses briefly, and the command
     * ends; `active()` true again before then resumes both. Interrupting it
     * (a shot, an auto step ending) skips the reverse.
     */
    fun collect(active: () -> Boolean): Command = logged(
        "Collect",
        deadline(
            feeder.intakeAssist(intaking = active, clock = clock),
            intake.collectWhile(active),
        ),
    )

    /** Intake and feeder backwards until interrupted. */
    fun eject(): Command = logged("Eject", parallel(intake.eject(), feeder.eject()))

    /**
     * Teleop shot: spins to `rpm()`; the first time the flywheel is at speed,
     * intake and feeder start together and keep feeding through the speed dip
     * of each ball. Ending it stops all three.
     */
    fun shoot(rpm: () -> Double): Command = logged(
        "Shoot",
        parallel(shooter.spinUp(rpm), feedThroughDips()),
    )

    /**
     * Auto shot, with the flywheel already held by the caller's `spinUp`:
     *  1. wait up to [ScoringConfig.readyTimeoutMs] for `ready()` (the caller's
     *     pose check and the flywheel speed). A timeout is logged, not fatal:
     *  2. feed. Intake and feeder still wait for flywheel speed, so a slow
     *     flywheel shoots late, never too slow. Feeding ends after
     *     [ScoringConfig.ballsPerBurst] detected shots plus
     *     [ScoringConfig.shotTailMs], or after [ScoringConfig.burstMaxMs].
     */
    fun burst(name: String, ready: () -> Boolean): Command {
        var baseline = 0
        return logged(
            "$name burst",
            sequential(
                instant { baseline = shooter.shotsFired },
                race(
                    waitUntil { ready() && shooter.atTarget },
                    sequential(
                        monotonicWaitMs(ScoringConfig.readyTimeoutMs, clock),
                        instant { recordEvent("AUTO: $name fed before ready (pose or speed)") },
                    ),
                ),
                race(
                    feedThroughDips(),
                    logged("$name burst time limit", monotonicWaitMs(ScoringConfig.burstMaxMs, clock)),
                    sequential(
                        waitUntil { shooter.shotsFired - baseline >= ScoringConfig.ballsPerBurst },
                        monotonicWaitMs(ScoringConfig.shotTailMs, clock),
                    ),
                ),
            ),
        )
    }

    /** Intake and feeder start together the first time the flywheel is at speed, then run through each dip. */
    private fun feedThroughDips(): Command = sequential(
        waitUntil { shooter.atTarget },
        parallel(
            intake.collect(),
            feeder.feedOnceReady(ready = { shooter.atTarget }, spinning = { shooter.targetRpm != null }),
        ),
    )
}

/** Auto shot timing, live-editable in Panels. Copy tuned values back here. */
@Configurable
object ScoringConfig {
    /** Longest an auto shot waits for pose and flywheel speed before feeding anyway. */
    @JvmField var readyTimeoutMs = 2000.0

    /** Balls the robot carries into a shot; the burst ends once this many dips are seen. */
    @JvmField var ballsPerBurst = 3

    /** Feeding after the last detected shot, so a ball already in the feeder still leaves. */
    @JvmField var shotTailMs = 300.0

    /** Upper bound on feeding, for when shots are missed by the dip detector. */
    @JvmField var burstMaxMs = 3000.0
}
