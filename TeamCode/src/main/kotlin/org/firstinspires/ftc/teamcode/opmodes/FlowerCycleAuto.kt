package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.api.Paths.curve
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.conditional
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShovelConfig
import org.firstinspires.ftc.teamcode.subsystems.ShovelSubsystem

/** Three timed bursts and two FLOWER collections using the supplied RED reference route. */
@Autonomous(name = "Flower Cycle RED", group = "Match")
open class FlowerCycleAutoRed : OpModeBase() {
    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    private lateinit var intake: IntakeSubsystem
    private lateinit var feeder: FeederSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var shovel: ShovelSubsystem
    private var shovelLowered = false
    private var startDelaySec = 0
    private var routine: Command? = null
    private var waitingToStart = false
    private var startedNs = 0L

    override val initialAlliance: Alliance get() = Alliance.RED

    // ---- USER PATH SECTION --------------------------------------------------
    // All coordinates are RED, inches and degrees. Alliance.poses() also handles BLUE.
    // Supplied reference route: first FLOWER -> far shot -> second FLOWER -> return shot.
    // Set pathsConfigured to false while replacing an incomplete route to refuse START.
    protected open val pathsConfigured = true
    protected val poses get() = alliance.poses()
    protected open val start get() = poses.of(56.0, 8.0, -90.0)
    protected open val firstFlower get() = poses.of(9.7698, 46.05, 180.0)
    protected open val farShot get() = poses.of(61.0083, 133.8026, 90.0)
    protected open val secondFlower get() = poses.of(46.7669, 131.9023, 90.0)
    protected open val homeShot get() = poses.of(61.3271, 133.0142, 90.0)

    protected open fun toFirstFlower(): Path = curve(
        start,
        poses.of(31.9419, 57.9825, 0.0),
        firstFlower,
    ).linear(start, firstFlower)

    protected open fun toFarShot(): Path = curve(
        firstFlower,
        poses.of(30.7763, 46.326, 0.0),
        poses.of(4.8116, 141.5, 0.0),
        poses.of(69.5153, 89.731, 0.0),
        farShot,
    ).linear(firstFlower, farShot)

    protected open fun toSecondFlower(): Path = curve(
        farShot,
        poses.of(61.2944, 93.3918, 0.0),
        poses.of(45.2203, 99.5701, 0.0),
        secondFlower,
    ).linear(farShot, secondFlower)

    protected open fun toHomeShot(): Path = curve(
        secondFlower,
        poses.of(54.8783, 78.0983, 0.0),
        homeShot,
    ).linear(secondFlower, homeShot)
    // ---- END USER PATH SECTION ----------------------------------------------

    override val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard +
            (RobotConfig.Intake.MOTORS + RobotConfig.Feeder.MOTORS + RobotConfig.Shooter.MOTORS)
                .map { Preflight.Requirement(it, DcMotorEx::class.java) } +
            listOf(RobotConfig.Shovel.LEFT_SERVO, RobotConfig.Shovel.RIGHT_SERVO)
                .map { Preflight.Requirement(it, ServoImplEx::class.java) }

    override fun configure() {
        val follower = Constants.create(hardwareMap) { robot.hubPower.batteryVolts }
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                isFollowing = drive::isFollowing,
                onFault = { routine?.let(Scheduler::cancel) },
                onStatusFault = {},
                startingPose = start,
            ),
        )
        intake = robot.register(IntakeSubsystem())
        feeder = robot.register(FeederSubsystem())
        shooter = robot.register(ShooterSubsystem())
        shovel = robot.register(ShovelSubsystem())
        shovel.defaultCommand = shovel.holdOffsetDegrees {
            if (shovelLowered) ShovelConfig.fullyLoweredOffsetDegrees else ShovelConfig.preset1OffsetDegrees
        }
    }

    private fun buildRoutine(): Command {
        check(pathsConfigured) { "Draw the Flower Cycle poses and four paths before running" }
        var driveOwned = false
        val steps = race(
            sequential(
                logged("Start delay", monotonicWaitMs(startDelaySec * 1000.0, robot.clock)),
                shootAt("Preload shot", start, { ShooterConfig.nearRpm }, FlowerCycleConfig.preloadFeedMs, null),
                logged("Lower shovels", instant { shovelLowered = true }),
                logged("Shovel settling", monotonicWaitMs(FlowerCycleConfig.shovelSettleMs, robot.clock)),
                collectAt("First FLOWER", toFirstFlower(), FlowerCycleConfig.firstCollectMs),
                shootAt("Far shot", farShot, { ShooterConfig.farRpm }, FlowerCycleConfig.farFeedMs, toFarShot()),
                collectAt("Second FLOWER", toSecondFlower(), FlowerCycleConfig.secondCollectMs),
                shootAt("Return shot", homeShot, { ShooterConfig.farRpm }, FlowerCycleConfig.homeFeedMs, toHomeShot()),
                instant { robot.recordEvent("AUTO: Flower Cycle complete") },
            ),
            logged("Routine time limit", monotonicWaitMs(29_000.0, robot.clock)),
        )
        return logged(
            "Flower Cycle routine",
            deadline(
                steps,
                // The routine owns the drive through its steps' requirements. This companion
                // only cleans up: completed holds otherwise survive cancellation of the group.
                logged(
                    "Routine drive cleanup",
                    Command.build()
                        .setStart { driveOwned = true }
                        .setEnd {
                            if (driveOwned) drive.stop()
                            driveOwned = false
                        },
                ),
            ),
        )
    }

    private fun followPath(name: String, path: Path): Command = race(
        drive.followCommand(path, holdEnd = true, name = name),
        logged("$name time limit", monotonicWaitMs(FlowerCycleConfig.pathTimeoutMs, robot.clock)),
    )

    /** Collect during travel, then dwell at the FLOWER. Cancelling the assist prevents a reverse pulse. */
    private fun collectAt(name: String, path: Path, collectMs: Double): Command = logged(
        "$name collection",
        deadline(
            sequential(
                followPath("Drive to $name", path),
                logged("$name intake dwell", monotonicWaitMs(collectMs, robot.clock)),
            ),
            intake.collect(),
            feeder.intakeAssist(intaking = { true }),
        ),
    )

    /** The same spin-up command spans the approach, settling, and the entire burst. */
    private fun shootAt(name: String, target: Pose, rpm: () -> Double, feedMs: Double, approach: Path?): Command {
        require(FlowerCycleConfig.spinUpProgress in 0.0..1.0) { "spinUpProgress must be in 0..1" }
        var warmup = false
        val travel = if (approach == null) instant {} else deadline(
            followPath("Drive to $name", approach),
            sequential(
                waitUntil { drive.pathProgress() >= FlowerCycleConfig.spinUpProgress },
                instant {
                    warmup = true
                    robot.recordEvent("AUTO: $name early flywheel spin-up")
                },
            ),
        )
        return logged(
            name,
            sequential(
                instant { warmup = approach == null },
                deadline(
                    sequential(
                        travel,
                        // A very short path can end between samples before the marker is seen.
                        instant { warmup = true },
                        drive.holdCommand(target, FlowerCycleConfig.settleTimeoutMs, "$name settle"),
                        logged(
                            "$name wait for pose and RPM",
                            race(
                                waitUntil { drive.atPose(target) && shooter.atTarget },
                                monotonicWaitMs(FlowerCycleConfig.readyTimeoutMs, robot.clock),
                            ),
                        ),
                        conditional(
                            { drive.atPose(target) && shooter.atTarget },
                            logged(
                                "$name feed burst",
                                deadline(
                                    monotonicWaitMs(feedMs, robot.clock),
                                    intake.collect(),
                                    feeder.feedOnceReady { true },
                                ),
                            ),
                            instant { robot.recordEvent("AUTO: $name skipped: pose or RPM not ready") },
                        ),
                    ),
                    sequential(waitUntil { warmup }, shooter.spinUp(rpm)),
                ),
            ),
        )
    }

    override fun onInitLoop() {
        if (driver.pressed(Button.DPAD_RIGHT)) startDelaySec = (startDelaySec + 1).coerceAtMost(10)
        if (driver.pressed(Button.DPAD_LEFT)) startDelaySec = (startDelaySec - 1).coerceAtLeast(0)
        telemetryBag.section("Auto") {
            put("paths configured", pathsConfigured)
            put("start delay s", startDelaySec)
            if (!pathsConfigured) put("setup", "Fill in USER PATH SECTION before START")
        }
    }

    override fun onStart() {
        if (!pathsConfigured) {
            abortAuto("Flower Cycle paths are not configured")
            return
        }
        if (!Constants.FORESIGHT_TUNED) {
            abortAuto("Foresight is not tuned")
            return
        }
        startedNs = robot.clock.nanos()
        waitingToStart = true
    }

    private fun startRoutineWhenReady() {
        localizer.fault?.let {
            waitingToStart = false
            abortAuto("pose lost before start: $it")
            return
        }
        if (!localizer.ready && (robot.clock.nanos() - startedNs) / 1e6 < 1000.0) return
        if (!localizer.ready) robot.recordEvent("AUTO: starting without a ready localizer: ${localizer.health()}")
        waitingToStart = false
        val selected = buildRoutine()
        routine = selected
        Scheduler.schedule(selected)
        if (!Scheduler.isScheduled(selected)) abortAuto("routine schedule rejected")
    }

    override fun onLoop() {
        if (waitingToStart) startRoutineWhenReady()
        val scheduled = routine?.let(Scheduler::isScheduled) ?: false
        telemetryBag.section("Auto") {
            put("alliance", alliance.name)
            put("pose", drive.pose)
            put("routine running", scheduled)
            put("shovel lowered", shovelLowered)
            put("target RPM", shooter.targetRpm ?: 0.0)
            put("measured RPM", shooter.measuredRpm)
        }
        if (routine != null && !scheduled) {
            routine = null
            robot.recordEvent("AUTO: routine no longer scheduled; stopping")
            requestOpModeStop()
        }
    }

    private fun abortAuto(reason: String) {
        robot.recordEvent("AUTO ABORTED: $reason")
        requestOpModeStop()
    }
}

@Autonomous(name = "Flower Cycle BLUE", group = "Match")
class FlowerCycleAutoBlue : FlowerCycleAutoRed() {
    override val initialAlliance: Alliance get() = Alliance.BLUE
}

/** Initial timing guesses: tune for the drawn paths, ball supply, and mechanism speed. */
@Configurable
object FlowerCycleConfig {
    @JvmField var preloadFeedMs = 1500.0
    @JvmField var farFeedMs = 1500.0
    @JvmField var homeFeedMs = 1500.0
    @JvmField var firstCollectMs = 700.0
    @JvmField var secondCollectMs = 700.0
    @JvmField var shovelSettleMs = 300.0
    @JvmField var spinUpProgress = 0.7
    @JvmField var pathTimeoutMs = 5000.0
    @JvmField var settleTimeoutMs = 1000.0
    @JvmField var readyTimeoutMs = 2000.0
}
