package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.logging.TelemetryBag
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Scoring
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShovelConfig
import org.firstinspires.ftc.teamcode.subsystems.ShovelSubsystem

/**
 * Preload shot, then two FLOWER collections each followed by a shot, along
 * [FlowerRoute]. Lifecycle (start delay, gates, limits, abort on a blocked
 * path) is [AutoBase]'s; shots and collections are [Scoring]'s. This file is
 * the order of steps and the two step shapes:
 *
 *  - [collectAt]: drive to a FLOWER; shovels lower once clear of the wall;
 *    intake and feeder start only within
 *    [FlowerCycleConfig.intakeStartDistanceIn] of the FLOWER (or on arrival),
 *    then dwell.
 *  - [shootAt]: drive to the shot pose; shovels rise; the flywheel spins up
 *    at [FlowerCycleConfig.spinUpProgress] of the approach and stays on
 *    through settling and the whole [Scoring.burst].
 */
@Autonomous(name = "Flower Cycle RED", group = "Match")
open class FlowerCycleAutoRed : AutoBase() {

    override val initialAlliance: Alliance get() = Alliance.RED
    override val routineName = "Flower Cycle"

    /** Set false while redrawing an incomplete route, to refuse START. */
    protected open val pathsConfigured = true

    /** The route; tests substitute a small course. */
    protected open val route: FlowerRoute get() = FlowerRoute(poses)

    override val startPose: Pose get() = route.start

    private lateinit var intake: IntakeSubsystem
    private lateinit var feeder: FeederSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var shovel: ShovelSubsystem
    private lateinit var scoring: Scoring

    /** The shovels' target; their default command follows it. */
    private var shovelLowered = false

    override val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard +
            (RobotConfig.Intake.MOTORS + RobotConfig.Feeder.MOTORS + RobotConfig.Shooter.MOTORS)
                .map { Preflight.Requirement(it, DcMotorEx::class.java) } +
            listOf(RobotConfig.Shovel.LEFT_SERVO, RobotConfig.Shovel.RIGHT_SERVO)
                .map { Preflight.Requirement(it, ServoImplEx::class.java) }

    override fun configureMechanisms() {
        intake = robot.register(IntakeSubsystem())
        feeder = robot.register(FeederSubsystem())
        shooter = robot.register(ShooterSubsystem(clock = robot.clock))
        shovel = robot.register(ShovelSubsystem())
        scoring = Scoring(intake, feeder, shooter, robot.clock, robot::recordEvent)
        shovel.defaultCommand = shovel.holdOffsetDegrees {
            if (shovelLowered) ShovelConfig.fullyLoweredOffsetDegrees else ShovelConfig.preset1OffsetDegrees
        }
    }

    override fun startBlocker(): String? =
        if (pathsConfigured) null else "Flower Cycle paths are not configured"

    override fun routine(): Command {
        check(pathsConfigured) { "Draw the Flower Cycle poses and paths before running" }
        val r = route
        val c = FlowerCycleConfig
        return sequential(
            shootAt("Preload shot", r.start, { ShooterConfig.nearRpm }),
            collectAt("First FLOWER", r.toFirstFlower(), r.firstFlower, c.firstCollectMs, c.shovelLowerDelayMs),
            shootAt("Far shot", r.farShot, { ShooterConfig.farRpm }, approach = r.toFarShot()),
            collectAt("Second FLOWER", r.toSecondFlower(), r.secondFlower, c.secondCollectMs, lowerDelayMs = 0.0),
            shootAt("Home shot", r.homeShot, { ShooterConfig.farRpm }, approach = r.toHomeShot()),
        )
    }

    /** Drive to [flower] along [path], lower the shovels once clear, collect near the end, then dwell. */
    private fun collectAt(name: String, path: Path, flower: Pose, dwellMs: Double, lowerDelayMs: Double): Command {
        var arrived = false
        return logged(
            "$name collection",
            deadline(
                sequential(
                    instant { arrived = false },
                    deadline(follow("Drive to $name", path), lowerShovelsWhenClear(name, lowerDelayMs)),
                    instant { arrived = true },
                    logged("$name intake dwell", monotonicWaitMs(dwellMs, robot.clock)),
                ),
                sequential(
                    waitUntil { arrived || within(flower, FlowerCycleConfig.intakeStartDistanceIn) },
                    scoring.collect { true },
                ),
            ),
        )
    }

    /** After [delayMs], and only once the robot is far enough from every wall that the shovels can't catch one. */
    private fun lowerShovelsWhenClear(name: String, delayMs: Double): Command = logged(
        "$name lower shovels after wall clearance",
        sequential(
            monotonicWaitMs(delayMs, robot.clock),
            waitUntil {
                val pose = drive.pose
                val length = RobotConfig.Field.LENGTH_INCHES
                minOf(pose.x(), pose.y(), length - pose.x(), length - pose.y()) >=
                    FlowerCycleConfig.shovelWallClearanceInches
            },
            instant { shovelLowered = true },
        ),
    )

    /**
     * Shoot from [target]. With an [approach], drive it first (shovels rise
     * after [FlowerCycleConfig.shovelRaiseDelayMs]; the flywheel starts at
     * [FlowerCycleConfig.spinUpProgress]); without one, shoot from here. One
     * spin-up command spans approach, settling and the burst.
     */
    private fun shootAt(name: String, target: Pose, rpm: () -> Double, approach: Path? = null): Command {
        require(FlowerCycleConfig.spinUpProgress in 0.0..1.0) { "spinUpProgress must be in 0..1" }
        var spinning = false
        val travel = if (approach == null) instant {} else deadline(
            follow("Drive to $name", approach),
            logged(
                "$name raise shovels after departure",
                sequential(monotonicWaitMs(FlowerCycleConfig.shovelRaiseDelayMs, robot.clock), instant { shovelLowered = false }),
            ),
            sequential(
                waitUntil { drive.pathProgress() >= FlowerCycleConfig.spinUpProgress },
                instant {
                    spinning = true
                    robot.recordEvent("AUTO: $name early flywheel spin-up")
                },
            ),
        )
        return logged(
            name,
            sequential(
                instant { spinning = approach == null },
                deadline(
                    sequential(
                        travel,
                        // Also on arrival, in case the approach was shorter than the raise delay.
                        instant { shovelLowered = false },
                        // A short approach can end before its spin-up marker is sampled.
                        instant { spinning = true },
                        drive.holdCommand(target, FlowerCycleConfig.settleTimeoutMs, "$name settle"),
                        scoring.burst(name) { drive.atPose(target) },
                    ),
                    sequential(waitUntil { spinning }, shooter.spinUp(rpm)),
                ),
            ),
        )
    }

    override fun TelemetryBag.Section.autoTelemetry() {
        put("shovel lowered", shovelLowered)
        put("target RPM", shooter.targetRpm ?: 0.0)
        put("measured RPM", shooter.measuredRpm)
        put("shots", shooter.shotsFired)
    }
}

@Autonomous(name = "Flower Cycle BLUE", group = "Match")
class FlowerCycleAutoBlue : FlowerCycleAutoRed() {
    override val initialAlliance: Alliance get() = Alliance.BLUE
}

/**
 * Flower Cycle timing and geometry, live-editable in Panels. Shot timing is
 * [org.firstinspires.ftc.teamcode.subsystems.ScoringConfig]; path and
 * routine limits are [AutoConfig].
 */
@Configurable
object FlowerCycleConfig {
    /** Intake dwell at each FLOWER after arrival. */
    @JvmField var firstCollectMs = 1000.0
    @JvmField var secondCollectMs = 1000.0

    /** Intake and feeder start this close to the FLOWER, not for the whole drive. */
    @JvmField var intakeStartDistanceIn = 24.0

    @JvmField var shovelLowerDelayMs = 1000.0
    @JvmField var shovelRaiseDelayMs = 500.0
    @JvmField var shovelWallClearanceInches = 18.0

    /** Fraction of a shot approach after which the flywheel starts. */
    @JvmField var spinUpProgress = 0.7

    /** Longest the hold at a shot pose waits for measured arrival. */
    @JvmField var settleTimeoutMs = 1000.0
}
