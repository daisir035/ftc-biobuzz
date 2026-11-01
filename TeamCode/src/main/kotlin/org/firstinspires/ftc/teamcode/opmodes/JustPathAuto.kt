package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.api.Paths.curve
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem

/**
 * Drives the user's four FLOWER curves, collecting only near each FLOWER. RED only.
 *
 * Every path turns along its length (`.linear`). The robot settles in the
 * final endpoint with a hold, then the op-mode stops.
 */
@Autonomous(name = "Just Path RED", group = "Diagnostics")
class JustPathAutoRed : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    private lateinit var intake: IntakeSubsystem
    // Picked on dpad left/right during init.
    private var startDelaySec = 0
    private var routine: Command? = null
    private var waitingToStart = false
    private var startedNs = 0L

    override val initialAlliance: Alliance get() = Alliance.RED

    // Poses (x, y, heading°) in RED coordinates.
    private val poses get() = alliance.poses()
    private val start get() = poses.of(57.3401, 7.4974, -90.0)
    private val firstFlower get() = poses.of(23.3387, 46.5526, 180.0)
    private val farShot get() = poses.of(66.0339, 115.8782, 90.0)
    private val secondFlower get() = poses.of(52.782, 112.4703, 90.0)
    private val finalShot get() = poses.of(65.18, 116.5975, 90.0)

    override val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard + RobotConfig.Intake.MOTORS.map {
            Preflight.Requirement(it, DcMotorEx::class.java)
        }

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
    }

    private fun path1(): Path = curve(
        start,
        poses.of(37.805, 49.4391, 0.0),
        firstFlower,
    ).linear(start, firstFlower)

    private fun path2(): Path = curve(
        firstFlower,
        poses.of(44.1777, 64.5854, 0.0),
        poses.of(0.9587, 90.9097, 0.0),
        poses.of(31.6563, 124.5746, 0.0),
        poses.of(69.5067, 96.8234, 0.0),
        farShot,
    ).linear(firstFlower, farShot)

    private fun path3(): Path = curve(
        farShot,
        poses.of(62.4671, 100.26, 0.0),
        poses.of(45.2203, 99.5701, 0.0),
        secondFlower,
    ).linear(farShot, secondFlower)

    private fun path4(): Path = curve(
        secondFlower,
        poses.of(55.5484, 86.4742, 0.0),
        finalShot,
    ).linear(secondFlower, finalShot)

    private fun buildRoutine(): Command = logged(
        "Just Path routine",
        race(
            sequential(
                logged("Start delay", monotonicWaitMs(startDelaySec * 1000.0, robot.clock)),
                collectAlong("Path 1 to first FLOWER", path1()),
                drive.followCommand(path2(), name = "Path 2 to far shot"),
                collectAlong("Path 3 to second FLOWER", path3()),
                drive.followCommand(path4(), name = "Path 4 to final shot"),
                drive.holdCommand(finalShot, name = "Settle at final shot"),
                instant { robot.recordEvent("AUTO: complete") },
            ),
            logged("Routine time limit", monotonicWaitMs(29_000.0, robot.clock)),
        ),
    )

    private fun collectAlong(name: String, path: Path): Command {
        require(JustPathConfig.intakeStartProgress in 0.0..1.0)
        var arrived = false
        return logged(name, sequential(
            instant { arrived = false },
            deadline(
                sequential(
                    drive.followCommand(path, holdEnd = true, name = "$name drive"),
                    instant { arrived = true },
                    logged("$name collect dwell", monotonicWaitMs(JustPathConfig.collectMs, robot.clock)),
                ),
                sequential(
                    waitUntil { arrived || drive.pathProgress() >= JustPathConfig.intakeStartProgress },
                    intake.collect(),
                ),
            ),
        ))
    }

    override fun onInitLoop() {
        if (driver.pressed(Button.DPAD_RIGHT)) startDelaySec = (startDelaySec + 1).coerceAtMost(10)
        if (driver.pressed(Button.DPAD_LEFT)) startDelaySec = (startDelaySec - 1).coerceAtLeast(0)
        telemetryBag.section("Auto") { put("start delay s", startDelaySec) }
    }

    override fun onStart() {
        if (!Constants.FORESIGHT_TUNED) {
            abortAuto("Foresight is not tuned")
            return
        }
        startedNs = robot.clock.nanos()
        waitingToStart = true
    }

    /** START pressed right after INIT: give the localizer up to a second, then go anyway unless the pose is lost. */
    private fun startRoutineWhenReady() {
        localizer.fault?.let {
            waitingToStart = false
            abortAuto("pose lost before start: $it")
            return
        }
        val waitedMs = (robot.clock.nanos() - startedNs) / 1e6
        if (!localizer.ready && waitedMs < 1_000.0) return
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
            put("mode", drive.driveModeName)
            put("routine running", scheduled)
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

@Configurable
object JustPathConfig {
    @JvmField var intakeStartProgress = 0.8
    @JvmField var collectMs = 700.0
}
