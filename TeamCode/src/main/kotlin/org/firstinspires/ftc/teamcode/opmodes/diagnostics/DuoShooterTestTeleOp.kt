package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem

/**
 * Bench test for the dual flywheel (both motors in [RobotConfig.Shooter.MOTORS]):
 * hold **R2** to hold [ShooterConfig.targetRpm] under the motor's
 * velocity PID, **dpad up/down** steps the target by ±100 RPM (also
 * live-editable in Panels), telemetry shows target vs measured RPM and the
 * at-speed flag. Velocity PIDF is tuned live in Panels
 * (`ShooterConfig.velocityP/I/D/F`, NaN keeps the hub default). No drivetrain
 * or localizer required; the WPILOG records `Shooter/…` channels, including
 * the applied PIDF, for analysis. For the single-flywheel bench motor
 * ([RobotConfig.Shooter.SINGLE_MOTOR]) use "Shooter Test" instead.
 */
@TeleOp(name = "Duo Shooter Test", group = "Diagnostics")
class DuoShooterTestTeleOp : OpModeBase() {

    private lateinit var shooter: ShooterSubsystem

    override val requiredDevices: List<Preflight.Requirement>
        get() = RobotConfig.Shooter.MOTORS.map { Preflight.Requirement(it, DcMotorEx::class.java) }
    override val publishFieldView: Boolean get() = false

    override fun configure() {
        shooter = robot.register(ShooterSubsystem())
        driver.trigger { driver.r2 > 0.1 }.whileTrue(shooter.spinUp())
        driver.button(Button.DPAD_UP).onTrue(instant { ShooterConfig.targetRpm += 100.0 })
        driver.button(Button.DPAD_DOWN).onTrue(instant { ShooterConfig.targetRpm -= 100.0 })
    }

    override fun onInitLoop() = tick()

    override fun onLoop() = tick()

    private fun tick() {
        telemetryBag.section("Shooter") {
            put("target rpm", ShooterConfig.targetRpm, decimals = 0)
            put("measured rpm", shooter.measuredRpm, decimals = 0)
            put("at speed", shooter.atTarget)
        }
    }
}
