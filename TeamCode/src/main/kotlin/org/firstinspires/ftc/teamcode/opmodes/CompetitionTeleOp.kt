package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Scoring
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
 *  - **R2** (hold) — shoot: the intake runs at once; the feeder
 *    starts the first time the flywheel is at speed and then feeds without
 *    pausing, so the whole burst goes out continuously
 *
 * Shooting takes the intake, so it overrides the driver's collect or eject;
 * a bumper still held when R2 is released takes the intake back.
 */
@TeleOp(name = "Competition", group = "Match")
class CompetitionTeleOp : TeleOpBase() {

    private enum class Preset { NEAR, FAR }

    private lateinit var intake: IntakeSubsystem
    private lateinit var feeder: FeederSubsystem
    private lateinit var shooter: ShooterSubsystem
    private lateinit var spinUp: Command
    private lateinit var scoring: Scoring

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
        shooter = robot.register(ShooterSubsystem(batteryVolts = { robot.hubPower.batteryVolts }))
        spinUp = shooter.spinUp { presetRpm }
        scoring = Scoring(intake, feeder, shooter)

        // Bumpers count only while not shooting, so releasing R2 with a bumper
        // held is a rising edge that restarts the driver's command.
        val shooting = { operator.r2 > SHOOT_THRESHOLD }
        driver.trigger { driver.held(Button.R1) && !shooting() }.whileTrue(intake.collect())
        driver.trigger { driver.held(Button.L1) && !shooting() }.whileTrue(intake.eject())

        operator.button(Button.DPAD_UP).onTrue(instant { preset = Preset.FAR })
        operator.button(Button.DPAD_DOWN).onTrue(instant { preset = Preset.NEAR })
        operator.trigger { shooting() }.whileTrue(scoring.feed())
    }

    override fun onLoop() {
        // A toggle, not a binding: onLoop() runs outside Scheduler.execute(), so this may schedule.
        if (operator.pressed(Button.CROSS)) {
            if (Scheduler.isScheduled(spinUp)) Scheduler.cancel(spinUp) else Scheduler.schedule(spinUp)
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

    private companion object {
        const val SHOOT_THRESHOLD = 0.3
    }
}
