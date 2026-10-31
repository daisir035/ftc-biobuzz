package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.GamepadEx.Button
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Preflight
import org.firstinspires.ftc.teamcode.subsystems.ShovelConfig
import org.firstinspires.ftc.teamcode.subsystems.ShovelSubsystem

/** Linked servo rotations from the measured aligned pair; this does not define a raised zero. */
@TeleOp(name = "Shovel Angle Test", group = "Diagnostics")
class ShovelAngleTestTeleOp : OpModeBase() {
    private lateinit var shovel: ShovelSubsystem

    override val requiredDevices: List<Preflight.Requirement>
        get() = listOf(RobotConfig.Shovel.LEFT_SERVO, RobotConfig.Shovel.RIGHT_SERVO)
            .map { Preflight.Requirement(it, ServoImplEx::class.java) }
    override val publishFieldView: Boolean get() = false

    override fun configure() {
        shovel = robot.register(ShovelSubsystem())
        driver.button(Button.L1).whileTrue(shovel.holdOffsetDegrees { ShovelConfig.targetOffsetDegrees })
    }

    override fun onInitLoop() = tick()
    override fun onLoop() = tick()

    private fun tick() {
        val c = ShovelConfig
        val step = c.stepDegrees / if (driver.held(Button.R1)) 5.0 else 1.0
        if (driver.pressed(Button.CROSS)) c.targetOffsetDegrees = 0.0
        if (driver.pressed(Button.CIRCLE)) c.targetOffsetDegrees = c.preset1OffsetDegrees
        if (driver.pressed(Button.SQUARE)) c.targetOffsetDegrees = c.preset2OffsetDegrees
        if (driver.pressed(Button.DPAD_UP)) c.targetOffsetDegrees += step
        if (driver.pressed(Button.DPAD_DOWN)) c.targetOffsetDegrees -= step

        telemetryBag.section("Shovel angle test") {
            put("reference", "Aligned pair only; NOT the fully raised zero")
            put("left reference", c.leftAlignedPosition, decimals = 4)
            put("right reference", c.rightAlignedPosition, decimals = 4)
            put("requested offset deg", c.targetOffsetDegrees, decimals = 1)
            put("applied offset deg", shovel.offsetDegrees, decimals = 1)
            put("left commanded position", shovel.leftPosition, decimals = 4)
            put("right commanded position", shovel.rightPosition, decimals = 4)
            put("PWM enabled", shovel.enabled)
            put("apply", "Hold L1 AFTER START; release disables PWM")
            put("presets", "Cross: aligned reference; Circle: preset 1; Square: preset 2")
            put("adjust", "Dpad up/down: +/- offset; hold R1: fine step")
            put("units", "Servo rotation at 300 deg / 500-2500 us; no measured angle feedback")
            put("limits", "Both stop together at either servo endpoint; check mechanical clearance")
        }
    }
}
