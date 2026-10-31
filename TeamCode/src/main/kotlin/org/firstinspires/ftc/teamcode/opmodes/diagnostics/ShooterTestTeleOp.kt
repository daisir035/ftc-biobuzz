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
 * Bench test for the single flywheel ([RobotConfig.Shooter.SINGLE_MOTOR]):
 * hold **R2** to hold [ShooterConfig.targetRpm] under the
 * flywheel's velocity PIDF, **dpad up/down** steps the target by ±100 RPM (also
 * live-editable in Panels), telemetry shows target vs measured RPM and the
 * at-speed flag. Gains are tuned live in Panels
 * (`ShooterConfig.kF/kP/kI/kD`). No drivetrain
 * or localizer required; the WPILOG records `Shooter/…` channels, including
 * the motor power, for analysis. For the dual mirrored flywheel use
 * "Duo Shooter Test" instead.
 */
@TeleOp(name = "Shooter Test", group = "Diagnostics")
class ShooterTestTeleOp : OpModeBase() {

    private lateinit var shooter: ShooterSubsystem

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(Preflight.Requirement(RobotConfig.Shooter.SINGLE_MOTOR, DcMotorEx::class.java))
    override val publishFieldView: Boolean get() = false

    override fun configure() {
        shooter = robot.register(ShooterSubsystem(listOf(RobotConfig.Shooter.SINGLE_MOTOR), feedbackMotor = RobotConfig.Shooter.SINGLE_MOTOR))
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
            put("power", shooter.power, decimals = 2)
        }
    }
}
