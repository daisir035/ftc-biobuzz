package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem

/**
 * Match teleop: drive plus intake, feeder and the dual flywheel.
 *
 * Driver (gamepad1), on top of [TeleOpBase]'s drive controls:
 *  - **R1** (hold) — intake collect
 *  - **L1** (hold) — intake eject
 *
 * Operator (gamepad2):
 *  - **Cross** — flywheel on/off at the selected preset
 *  - **dpad up / down** — far / near preset ([ShooterConfig.farRpm] / [ShooterConfig.nearRpm])
 *  - **R2** (hold) — shoot: intake and feeder wait until the flywheel first
 *    reaches speed, then run without pausing until released
 *
 * Shooting takes the intake, so it overrides the driver's collect until released.
 */
@TeleOp(name = "Competition", group = "Match")
class CompetitionTeleOp : TeleOpBase() {

    private enum class Preset { NEAR, FAR }

    private lateinit var intake: IntakeSubsystem
    private lateinit var feeder: FeederSubsystem
    private lateinit var shooter: ShooterSubsystem

    private var preset = Preset.NEAR

    private val presetRpm: Double
        get() = when (preset) {
            Preset.NEAR -> ShooterConfig.nearRpm
            Preset.FAR -> ShooterConfig.farRpm
        }

    override val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard +
            (RobotConfig.Intake.MOTORS + RobotConfig.Feeder.MOTORS + RobotConfig.Shooter.MOTORS)
                .map { Preflight.Requirement(it, DcMotorEx::class.java) }

    override fun configureTeleop() {
        intake = robot.register(IntakeSubsystem())
        feeder = robot.register(FeederSubsystem())
        shooter = robot.register(ShooterSubsystem())

        driver.button(Button.R1).whileTrue(intake.collect())
        driver.button(Button.L1).whileTrue(intake.eject())

        operator.button(Button.CROSS).toggleOnTrue(shooter.spinUp { presetRpm })
        operator.button(Button.DPAD_UP).onTrue(instant { preset = Preset.FAR })
        operator.button(Button.DPAD_DOWN).onTrue(instant { preset = Preset.NEAR })
        operator.trigger { operator.r2 > 0.3 }.whileTrue(shoot())
    }

    private fun shoot(): Command = logged(
        "Shoot",
        sequential(
            waitUntil { shooter.atTarget },
            parallel(intake.collect(), feeder.feedOnceReady { true }),
        ),
    )

    override fun onLoop() {
        telemetryBag.section("Shooter") {
            put("preset", preset)
            put("target rpm", shooter.targetRpm ?: 0.0, decimals = 0)
            put("measured rpm", shooter.measuredRpm, decimals = 0)
            put("at speed", shooter.atTarget)
            put("power", shooter.power, decimals = 2)
        }
        telemetryBag.section("Drive") {
            put("pose", drive.pose)
            put("fieldCentric", drive.fieldCentric)
        }
    }
}
