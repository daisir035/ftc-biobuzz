package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.Paths.line
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem

/**
 * Default auton: one straight path of about 20 in from the start pose to
 * [path1End], turning along the way (`linear`), then a hold until the
 * measured pose arrives (the path's parametric end is not arrival). RED
 * coordinates; BLUE is the season's 180° rotation ([Alliance.poses]).
 *
 * Lifecycle follows the Example Auto skeleton: start pose is written to the
 * Pinpoint at INIT (place the robot first), start delay is picked on dpad
 * left/right during init, start is refused without Foresight tuning, the
 * localizer gets up to [LOCALIZER_WAIT_MS] to become ready (then the routine
 * runs anyway; only a lost pose cancels it), and the op-mode stops when the
 * routine is no longer scheduled.
 */
@Autonomous(name = "Default Auto RED", group = "Match")
open class DefaultAutoRed : OpModeBase() {

    private lateinit var drive: MecanumDriveSubsystem
    private lateinit var localizer: LocalizerSubsystem
    // Picked on dpad left/right during init, agreed with the alliance partner before the match.
    private var startDelaySec = 0
    private var routine: Command? = null
    private var waitingToStart = false
    private var startedNs = 0L

    /** The BLUE copy of this file overrides this and changes nothing else. */
    override val initialAlliance: Alliance get() = Alliance.RED

    // Poses (x, y, heading°) in RED coordinates.
    private val poses get() = alliance.poses()
    private val start get() = poses.of(56.0, 8.0, 90.0)
    private val path1End get() = poses.of(70.7572, 21.5438, 180.0)

    override fun configure() {
        val follower = Constants.create(hardwareMap) { robot.hubPower.batteryVolts }
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                isFollowing = drive::isFollowing,
                // Driving blind is worse than parking: cancel the routine (its
                // drive command stops the follower) and stay put.
                onFault = { routine?.let(Scheduler::cancel) },
                // The pose is still live: keep driving, the event is in the log.
                onStatusFault = {},
                // Written to the Pinpoint at INIT, after the reset. Place the robot before INIT.
                startingPose = start,
            ),
        )
    }

    private fun path1(): Path = line(start, path1End).linear(start, path1End)

    // Named steps show up in commands/events and commands/active. The drive
    // factories are already logged (pass `name`); other steps are wrapped with
    // logged().
    private fun buildRoutine(): Command = logged(
        "Default Auto routine",
        race(
            sequential(
                logged("Start delay", waitMs(startDelaySec * 1000.0)),
                race(
                    drive.followCommand(path1(), name = "Drive path 1"),
                    logged("Path 1 time limit", waitMs(8_000.0)),
                ),
                drive.holdCommand(path1End, name = "Settle at path 1 end"),
                instant { robot.recordEvent("AUTO: complete") },
            ),
            logged("Routine time limit", waitMs(29_000.0)),
        ),
    )

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

    /** START pressed right after INIT: give the localizer up to [LOCALIZER_WAIT_MS], then go anyway unless the pose is lost. */
    private fun startRoutineWhenReady() {
        localizer.fault?.let {
            waitingToStart = false
            abortAuto("pose lost before start: $it")
            return
        }
        val waitedMs = (robot.clock.nanos() - startedNs) / 1e6
        if (!localizer.ready && waitedMs < LOCALIZER_WAIT_MS) return
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

    private companion object {
        const val LOCALIZER_WAIT_MS = 1_000.0
    }
}

@Autonomous(name = "Default Auto BLUE", group = "Match")
class DefaultAutoBlue : DefaultAutoRed() {
    override val initialAlliance: Alliance get() = Alliance.BLUE
}
