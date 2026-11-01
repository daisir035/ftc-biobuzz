package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitUntil
import com.pedropathing.ivy.groups.Groups.parallel
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.core.logging.logged
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShovelConfig
import org.firstinspires.ftc.teamcode.subsystems.ShovelSubsystem

/**
 * Single-driver match teleop: drive, intake, feeder, flywheel and shovels.
 *
 * Driver (gamepad1), on top of [TeleOpBase]'s drive controls:
 *  - **R1** (hold) — intake collect with low-power feeder assist; release
 *    stops the intake and gives the feeder a delayed reverse pulse
 *  - **L1** (hold) — intake eject
 *
 *  - **Cross** — flywheel on/off at the selected preset
 *  - **dpad up / down** — far / near preset ([ShooterConfig.farRpm] / [ShooterConfig.nearRpm])
 *  - **R2** (hold) — spin up the flywheel and shoot once at speed;
 *    release stops the flywheel, intake and feeder
 *  - **L2** (hold) — precision driving
 *  - **Square** — switch shovels between the 0° and 125° working presets
 *
 * Shooting takes the intake, so it overrides the driver's collect until released.
 */
@TeleOp(name = "Competition", group = "Match")
class CompetitionTeleOp : TeleOpBase() {

    private enum class Preset { NEAR, FAR }

    private lateinit var intake: IntakeSubsystem
    private lateinit var feeder: FeederSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var shovel: ShovelSubsystem
    private var shovelExtended = false

    override val precisionRequested: Boolean get() = driver.l2 > 0.1

    private var preset = Preset.NEAR

    private val presetRpm: Double
        get() = when (preset) {
            Preset.NEAR -> ShooterConfig.nearRpm
            Preset.FAR -> ShooterConfig.farRpm
        }

    override val requiredDevices: List<Preflight.Requirement>
        get() = Preflight.standard +
            (RobotConfig.Intake.MOTORS + RobotConfig.Feeder.MOTORS + RobotConfig.Shooter.MOTORS)
                .map { Preflight.Requirement(it, DcMotorEx::class.java) } +
            listOf(RobotConfig.Shovel.LEFT_SERVO, RobotConfig.Shovel.RIGHT_SERVO)
                .map { Preflight.Requirement(it, ServoImplEx::class.java) }

    override fun configureTeleop() {
        intake = robot.register(IntakeSubsystem())
        feeder = robot.register(FeederSubsystem())
        shooter = robot.register(ShooterSubsystem())
        shovel = robot.register(ShovelSubsystem())
        shovel.defaultCommand = shovel.holdOffsetDegrees {
            if (shovelExtended) ShovelConfig.preset2OffsetDegrees else ShovelConfig.preset1OffsetDegrees
        }

        driver.button(Button.R1).whileTrue(intake.collect())
            .onTrue(feeder.intakeAssist(intaking = { driver.held(Button.R1) }))
        driver.button(Button.L1).whileTrue(intake.eject())

        driver.button(Button.CROSS).toggleOnTrue(shooter.spinUp { presetRpm })
        driver.button(Button.DPAD_UP).onTrue(instant { preset = Preset.FAR })
        driver.button(Button.DPAD_DOWN).onTrue(instant { preset = Preset.NEAR })
        driver.trigger { driver.r2 > 0.3 }.whileTrue(shoot())
        driver.button(Button.SQUARE).onTrue(instant { shovelExtended = !shovelExtended })
    }

    private fun shoot(): Command = logged(
        "Shoot",
        parallel(
            shooter.spinUp { presetRpm },
            sequential(
                waitUntil { shooter.atTarget },
                parallel(intake.collect(), feeder.feedOnceReady { true }),
            ),
        ),
    )

    override fun onLoop() {
        telemetryBag.section("Shovel") {
            put("preset", if (shovelExtended) 2 else 1)
            put("offset deg", shovel.offsetDegrees, decimals = 1)
            put("fully lowered deg", ShovelConfig.fullyLoweredOffsetDegrees, decimals = 1)
        }
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
