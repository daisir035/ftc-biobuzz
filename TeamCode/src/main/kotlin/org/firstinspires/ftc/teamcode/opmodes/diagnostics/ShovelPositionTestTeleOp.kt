package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import java.util.Locale
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.subsystems.ShovelSubsystem

/** Captures visually verified shovel positions; no drive, localization or automatic motion. */
@TeleOp(name = "Shovel Position Test", group = "Diagnostics")
class ShovelPositionTestTeleOp : OpModeBase() {
    private lateinit var shovel: ShovelSubsystem
    private val captures = linkedMapOf<String, String>()
    private var status = "START, then hold L1 to apply targets. Release L1 to disable PWM."

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(RobotConfig.Shovel.LEFT_SERVO, RobotConfig.Shovel.RIGHT_SERVO)
            .map { Preflight.Requirement(it, ServoImplEx::class.java) }
    override val publishFieldView: Boolean get() = false

    override fun configure() {
        shovel = robot.register(ShovelSubsystem())
        driver.button(Button.L1).whileTrue(shovel.holdPositions(
            { ShovelCalibrationConfig.leftPosition },
            { ShovelCalibrationConfig.rightPosition },
        ))
        driver.button(Button.CROSS).onTrue(instant { capture("Raised") })
        driver.button(Button.CIRCLE).onTrue(instant { capture("Down 60") })
        driver.button(Button.SQUARE).onTrue(instant { capture("Down 70") })
    }

    private fun capture(name: String) {
        if (!shovel.enabled || !driver.held(Button.L1)) {
            status = "Hold L1 and visually verify the position before capturing."
            return
        }
        val pair = String.format(Locale.US, "left = %.4f, right = %.4f", shovel.leftPosition, shovel.rightPosition)
        captures[name] = pair
        status = "Captured $name"
        robot.recordEvent("SHOVEL CAPTURE: $name: $pair (left REVERSE, right FORWARD, PWM 500-2500 us)")
    }

    override fun onInitLoop() = tick()
    override fun onLoop() = tick()

    private fun tick() {
        val c = ShovelCalibrationConfig
        val step = c.step / if (driver.held(Button.R1)) 5.0 else 1.0
        if (driver.pressed(Button.DPAD_LEFT)) c.leftPosition = (c.leftPosition - step).coerceIn(0.0, 1.0)
        if (driver.pressed(Button.DPAD_RIGHT)) c.leftPosition = (c.leftPosition + step).coerceIn(0.0, 1.0)
        if (driver.pressed(Button.DPAD_DOWN)) c.rightPosition = (c.rightPosition - step).coerceIn(0.0, 1.0)
        if (driver.pressed(Button.DPAD_UP)) c.rightPosition = (c.rightPosition + step).coerceIn(0.0, 1.0)
        telemetryBag.section("Shovel calibration") {
            put("status", status)
            put("PWM enabled", shovel.enabled)
            put("left target", c.leftPosition, decimals = 4)
            put("right target", c.rightPosition, decimals = 4)
            put("step", step, decimals = 4)
            put("directions", "left REVERSE / right FORWARD; PWM 500-2500 us")
            put("apply", "Hold L1 AFTER START; initial 0.5000 is NOT the measured position")
            put("adjust", "Dpad left/right: left servo -/+; down/up: right servo -/+; hold R1: fine")
            put("capture", "Cross: raised; Circle: down 60; Square: down 70 (labels, not measured angles)")
            put("results", "Copy before STOP; captures also appear in WPILOG events")
        }
        telemetryBag.section("Captured positions") {
            for (name in listOf("Raised", "Down 60", "Down 70")) put(name, captures[name] ?: "not captured")
        }
    }
}

/** Initial targets only; they do not identify the physical raised position. */
@Configurable
object ShovelCalibrationConfig {
    @JvmField var leftPosition = 0.5
    @JvmField var rightPosition = 0.5
    @JvmField var step = 0.005
}
