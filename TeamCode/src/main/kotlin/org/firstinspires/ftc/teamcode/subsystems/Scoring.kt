package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import org.firstinspires.ftc.teamcode.core.logging.logged

/**
 * Commands that span intake, feeder and shooter, shared by teleop and autos so
 * both shoot the same way. Not a subsystem: it owns no hardware, and each
 * command requires the subsystems it drives.
 */
class Scoring(
    private val intake: IntakeSubsystem,
    private val feeder: FeederSubsystem,
    private val shooter: ShooterSubsystem,
) {

    /**
     * Intake runs at once; the feeder starts the first time the flywheel is at
     * speed and then feeds without pausing. Needs the flywheel spun up by
     * another command. Never finishes by itself (teleop holds it on a trigger).
     */
    fun feed(): Command = logged(
        "Shoot",
        parallel(intake.collect(), feeder.feedOnceReady(ready = { shooter.atTarget }, spinning = { shooter.targetRpm != null })),
    )

    /**
     * A bounded burst for autos: spin to [rpm], wait for speed (at most
     * [ScoringConfig.spinUpTimeoutMs]), then [feed] for [ScoringConfig.burstMs].
     * The flywheel stops when it ends. A spin-up timeout is not a failure: the
     * feeder still waits for speed inside the burst window, so a slow flywheel
     * shoots late or not at all, never at the wrong speed.
     */
    fun burst(rpm: () -> Double): Command = logged(
        "Shoot burst",
        deadline(
            sequential(
                race(waitUntil { shooter.atTarget }, waitMs(ScoringConfig.spinUpTimeoutMs)),
                race(feed(), waitMs(ScoringConfig.burstMs)),
            ),
            shooter.spinUp(rpm),
        ),
    )
}

/** Auto burst timing, live-editable in Panels. Placeholders until bursts are timed on the robot. */
@Configurable
object ScoringConfig {
    /** Longest an auto waits for the flywheel to reach speed before the burst window opens. */
    @JvmField var spinUpTimeoutMs = 1500.0

    /** How long an auto burst feeds; long enough to empty the robot. */
    @JvmField var burstMs = 1500.0
}
