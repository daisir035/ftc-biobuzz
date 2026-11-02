package org.firstinspires.ftc.teamcode.opmodes

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.deadline
import com.pedropathing.ivy.groups.Groups.race
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import kotlin.math.hypot
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.logging.TelemetryBag
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem

/**
 * Everything an autonomous does that is not its route, so a routine file is
 * only poses, paths and steps (the same split as [TeleOpBase]):
 *
 *  - Drive first, localizer second; [startPose] is written to the Pinpoint at
 *    INIT (place the robot first). A lost pose cancels the routine and the
 *    robot parks; Pinpoint status problems are only logged.
 *  - Start delay on dpad left/right during init, agreed with the partner.
 *  - START is refused without Foresight tuning or when [startBlocker] says
 *    so; the localizer gets up to [LOCALIZER_WAIT_MS] to confirm the pose,
 *    then the routine runs anyway.
 *  - The routine runs inside the start delay, a [AutoConfig.routineLimitMs]
 *    limit and [abortRoutine]; when it ends for any reason the drive stops
 *    (a finished hold would otherwise keep holding) and the op-mode stops.
 *
 * One `@Autonomous` class per alliance and routine; the BLUE copy overrides
 * [initialAlliance] only.
 */
abstract class AutoBase : OpModeBase() {

    protected lateinit var drive: MecanumDriveSubsystem
        private set
    protected lateinit var localizer: LocalizerSubsystem
        private set

    /** RED coordinates (x, y, heading°) mapped to this alliance. */
    protected val poses get() = alliance.poses()

    /** Where the robot is placed before INIT. */
    protected abstract val startPose: Pose

    /** Shown in `commands/events` as "<name> routine". */
    protected abstract val routineName: String

    /** Register the mechanisms this routine uses; drive and localizer exist already. */
    protected open fun configureMechanisms() {}

    /** The steps, as plain Ivy groups of drive and mechanism commands. Built at START. */
    protected abstract fun routine(): Command

    /** A reason START must be refused (e.g. an unfinished route), or null. */
    protected open fun startBlocker(): String? = null

    /** Extra lines for the "Auto" telemetry section. */
    protected open fun TelemetryBag.Section.autoTelemetry() {}

    private var startDelaySec = 0
    private var scheduled: Command? = null
    private var waitingToStart = false
    private var startedNs = 0L
    private var abortReason: String? = null

    final override fun configure() {
        val follower = Constants.create(hardwareMap) { robot.hubPower.batteryVolts }
        drive = robot.register(MecanumDriveSubsystem(follower))
        localizer = robot.register(
            LocalizerSubsystem(
                follower,
                isFollowing = drive::isFollowing,
                // Driving blind is worse than parking: cancel the routine (its
                // drive command stops the follower) and stay put.
                onFault = { scheduled?.let(Scheduler::cancel) },
                // The pose is still live: keep driving, the event is in the log.
                onStatusFault = {},
                startingPose = startPose,
            ),
        )
        configureMechanisms()
    }

    /** The whole run: delay, [routine], limits and drive cleanup. */
    internal fun buildRoutine(): Command {
        // First, so an unfinished route fails here, before anything else is built.
        val body = routine()
        abortReason = null
        var driveOwned = false
        val steps = race(
            sequential(
                logged("Start delay", monotonicWaitMs(startDelaySec * 1000.0, robot.clock)),
                body,
                instant { robot.recordEvent("AUTO: $routineName complete") },
            ),
            logged("Routine time limit", monotonicWaitMs(AutoConfig.routineLimitMs, robot.clock)),
            logged("Stop routine after abort", waitUntil { abortReason != null }),
        )
        return logged(
            "$routineName routine",
            deadline(
                steps,
                // Owns nothing; only stops the drive when the routine ends. A hold
                // that finished naturally keeps holding after its group is cancelled.
                Command.build()
                    .setStart { driveOwned = true }
                    .setEnd {
                        if (driveOwned) drive.stop()
                        driveOwned = false
                    },
            ),
        )
    }

    // ------------------------------------------------------------ step helpers

    /**
     * Follow [path] and hold its end. If [AutoConfig.pathTimeoutMs] passes
     * before the robot is at the end pose (blocked, pushed), the routine is
     * aborted instead of driving the next path from the wrong place.
     */
    protected fun follow(name: String, path: Path): Command {
        var timedOut = false
        return sequential(
            instant { timedOut = false },
            race(
                drive.followCommand(path, holdEnd = true, name = name),
                logged(
                    "$name time limit",
                    sequential(monotonicWaitMs(AutoConfig.pathTimeoutMs, robot.clock), instant { timedOut = true }),
                ),
            ),
            instant {
                if (timedOut && !drive.atPose(path.endPose())) {
                    abortRoutine("$name timed out before arrival at ${drive.pose}")
                }
            },
        )
    }

    /** True once the measured pose is within [inches] of [target] (position only). */
    protected fun within(target: Pose, inches: Double): Boolean {
        val pose = drive.pose
        return hypot(pose.x() - target.x(), pose.y() - target.y()) <= inches
    }

    /** Ends the routine after this tick; the drive stops and the op-mode ends. */
    protected fun abortRoutine(reason: String) {
        if (abortReason != null) return
        abortReason = reason
        drive.stop()
        robot.recordEvent("AUTO ABORTED: $reason")
    }

    // ---------------------------------------------------------------- lifecycle

    override fun onInitLoop() {
        if (driver.pressed(Button.DPAD_RIGHT)) startDelaySec = (startDelaySec + 1).coerceAtMost(10)
        if (driver.pressed(Button.DPAD_LEFT)) startDelaySec = (startDelaySec - 1).coerceAtLeast(0)
        telemetryBag.section("Auto") {
            put("start delay s", startDelaySec)
            startBlocker()?.let { put("cannot start", it) }
        }
    }

    override fun onStart() {
        val blocked = startBlocker() ?: if (!Constants.FORESIGHT_TUNED) "Foresight is not tuned" else null
        if (blocked != null) {
            stopAuto("AUTO ABORTED: $blocked")
            return
        }
        startedNs = robot.clock.nanos()
        waitingToStart = true
    }

    /** START pressed right after INIT: give the localizer a moment, then go anyway unless the pose is lost. */
    private fun startRoutineWhenReady() {
        localizer.fault?.let {
            waitingToStart = false
            stopAuto("AUTO ABORTED: pose lost before start: $it")
            return
        }
        val waitedMs = (robot.clock.nanos() - startedNs) / 1e6
        if (!localizer.ready && waitedMs < LOCALIZER_WAIT_MS) return
        if (!localizer.ready) robot.recordEvent("AUTO: starting without a ready localizer: ${localizer.health()}")
        waitingToStart = false
        val selected = buildRoutine()
        scheduled = selected
        Scheduler.schedule(selected)
        if (!Scheduler.isScheduled(selected)) stopAuto("AUTO ABORTED: routine schedule rejected")
    }

    override fun onLoop() {
        if (waitingToStart) startRoutineWhenReady()
        val running = scheduled?.let(Scheduler::isScheduled) ?: false
        telemetryBag.section("Auto") {
            put("alliance", alliance.name)
            put("pose", drive.pose)
            put("mode", drive.driveModeName)
            put("routine running", running)
            abortReason?.let { put("aborted", it) }
            autoTelemetry()
        }
        if (scheduled != null && !running) {
            scheduled = null
            stopAuto("AUTO: routine no longer scheduled; stopping")
        }
    }

    private fun stopAuto(event: String) {
        robot.recordEvent(event)
        requestOpModeStop()
    }

    private companion object {
        const val LOCALIZER_WAIT_MS = 1_000.0
    }
}

/** Limits shared by every autonomous, live-editable in Panels. */
@Configurable
object AutoConfig {
    /** The whole routine, start delay included; inside the 30 s period with margin. */
    @JvmField var routineLimitMs = 29_000.0

    /** One path, from start to measured arrival, before the routine is aborted. */
    @JvmField var pathTimeoutMs = 5_000.0
}
