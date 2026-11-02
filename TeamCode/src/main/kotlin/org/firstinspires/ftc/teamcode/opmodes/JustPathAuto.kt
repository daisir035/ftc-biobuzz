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
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.core.monotonicWaitMs
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem

/**
 * Drives [FlowerRoute] with no shooting: a pathing test. The intake runs
 * only near each FLOWER. RED only. Ends with a hold at the home shot pose.
 */
@Autonomous(name = "Just Path RED", group = "Diagnostics")
class JustPathAutoRed : AutoBase() {

    override val initialAlliance: Alliance get() = Alliance.RED
    override val routineName = "Just Path"

    private val route get() = FlowerRoute(poses)
    override val startPose: Pose get() = route.start

    private lateinit var intake: IntakeSubsystem

    override val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard + RobotConfig.Intake.MOTORS.map {
            Preflight.Requirement(it, DcMotorEx::class.java)
        }

    override fun configureMechanisms() {
        intake = robot.register(IntakeSubsystem())
    }

    override fun routine(): Command {
        val r = route
        return sequential(
            collectAlong("Path 1 to first FLOWER", r.toFirstFlower()),
            follow("Path 2 to far shot", r.toFarShot()),
            collectAlong("Path 3 to second FLOWER", r.toSecondFlower()),
            follow("Path 4 to home shot", r.toHomeShot()),
            drive.holdCommand(r.homeShot, name = "Settle at home shot"),
        )
    }

    /** Follow [path]; the intake starts at [JustPathConfig.intakeStartProgress] (or arrival) and runs through a dwell. */
    internal fun collectAlong(name: String, path: Path): Command {
        require(JustPathConfig.intakeStartProgress in 0.0..1.0)
        var arrived = false
        return logged(
            name,
            sequential(
                instant { arrived = false },
                deadline(
                    sequential(
                        follow("$name drive", path),
                        instant { arrived = true },
                        logged("$name collect dwell", monotonicWaitMs(JustPathConfig.collectMs, robot.clock)),
                    ),
                    sequential(
                        waitUntil { arrived || drive.pathProgress() >= JustPathConfig.intakeStartProgress },
                        intake.collect(),
                    ),
                ),
            ),
        )
    }
}

@Configurable
object JustPathConfig {
    @JvmField var intakeStartProgress = 0.8
    @JvmField var collectMs = 700.0
}
