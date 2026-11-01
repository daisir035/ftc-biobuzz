package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.core.GamepadEx
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.firstinspires.ftc.teamcode.subsystems.FeederConfig
import org.firstinspires.ftc.teamcode.subsystems.IntakeConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShovelConfig
import org.firstinspires.ftc.teamcode.subsystems.ShovelSubsystem
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class SingleDriverCompetitionTest {
    private val saved = listOf(ShooterConfig, ShovelConfig).map(::ConfigSnapshot)
    private val intake = MotorProbe()
    private val feeder = MotorProbe()
    private val up = MotorProbe()
    private val hardware = FakeHardwareMap().apply {
        put("intake", intake.device)
        put("feeder", feeder.device)
        put("up", up.device)
        put("down", MotorProbe().device)
    }
    private val robot = Robot(hardware)
    private val driver = GamepadEx(Gamepad())
    private val operator = GamepadEx(Gamepad())
    private val teleop = CompetitionTeleOp()

    init {
        for ((name, value) in listOf("robot" to robot, "driver" to driver, "operator" to operator)) {
            OpModeBase::class.java.getDeclaredField(name).apply { isAccessible = true }.set(teleop, value)
        }
        CompetitionTeleOp::class.java.getDeclaredMethod("configureTeleop")
            .apply { isAccessible = true }.invoke(teleop)
        // Servo hardware is covered separately; exercise real bindings and staged shovel targets here.
        robot.subsystems().filterNot { it is ShovelSubsystem }.forEach { it.init(hardware) }
        driver.lockBindings()
        operator.lockBindings()
    }

    @After
    fun tearDown() {
        robot.stop()
        saved.forEach { it.restore() }
    }

    private fun tick() = robot.loop(input = { driver.update(); operator.update() })

    @Test
    fun ejectReversesIntakeAndFeederAndReleaseStopsBoth() {
        driver.raw.right_bumper = true
        tick()
        assertEquals(FeederConfig.intakePower, feeder.power, 0.0)

        driver.raw.left_bumper = true
        tick()
        assertEquals(IntakeConfig.ejectPower, intake.power, 0.0)
        assertTrue(feeder.power < 0.0)

        driver.raw.right_bumper = false
        repeat(4) { tick() }
        assertEquals(IntakeConfig.ejectPower, intake.power, 0.0)
        assertTrue(feeder.power < 0.0)

        driver.raw.left_bumper = false
        tick()
        assertEquals(0.0, intake.power, 0.0)
        assertEquals(0.0, feeder.power, 0.0)
    }

    @Test
    fun gamepadOneControlsShooterPresetsFiringAndBothShovelPositions() {
        ShooterConfig.nearRpm = 1200.0
        ShooterConfig.farRpm = 1500.0
        ShovelConfig.preset1OffsetDegrees = 0.0
        ShovelConfig.preset2OffsetDegrees = 125.0
        driver.raw.cross = true
        tick()
        val shooter = robot.subsystems().filterIsInstance<ShooterSubsystem>().single()
        assertEquals(1200.0, shooter.targetRpm!!, 0.0)

        operator.raw.cross = true
        driver.raw.dpad_up = true
        tick()
        assertEquals(1500.0, shooter.targetRpm!!, 0.0)

        up.velocity = 2595.0 // 1500 RPM at 103.8 ticks/rev.
        driver.raw.right_trigger = 1.0f
        repeat(4) { tick() }
        assertEquals(1.0, intake.power, 0.0)
        assertEquals(FeederConfig.feedPower, feeder.power, 0.0)
        driver.raw.right_trigger = 0.0f
        tick()
        assertEquals(0.0, intake.power, 0.0)
        assertEquals(0.0, feeder.power, 0.0)

        driver.raw.square = true
        tick()
        val shovel = robot.subsystems().filterIsInstance<ShovelSubsystem>().single()
        assertEquals(125.0, shovel.offsetDegrees, 0.0)
        driver.raw.square = false
        tick()
        driver.raw.square = true
        tick()
        assertEquals(0.0, shovel.offsetDegrees, 0.0)
    }

    @Test
    fun rightTriggerStartsTheFlywheelFromRestAndReleaseStopsIt() {
        ShooterConfig.nearRpm = 1200.0
        val shooter = robot.subsystems().filterIsInstance<ShooterSubsystem>().single()
        tick()
        assertNull(shooter.targetRpm)

        driver.raw.right_trigger = 1.0f
        repeat(4) { tick() }
        assertEquals(1200.0, shooter.targetRpm!!, 0.0)
        assertTrue(up.power > 0.0)
        assertEquals(0.0, intake.power, 0.0)
        assertEquals(0.0, feeder.power, 0.0)

        driver.raw.right_trigger = 0.0f
        tick()
        assertNull(shooter.targetRpm)
        assertEquals(0.0, up.power, 0.0)

        driver.raw.right_trigger = 1.0f
        repeat(4) { tick() }
        assertEquals(1200.0, shooter.targetRpm!!, 0.0)
        up.velocity = 2076.0 // 1200 RPM at 103.8 ticks/rev.
        repeat(4) { tick() }
        assertEquals(1.0, intake.power, 0.0)
        assertEquals(FeederConfig.feedPower, feeder.power, 0.0)

        driver.raw.right_trigger = 0.0f
        tick()
        assertNull(shooter.targetRpm)
        assertEquals(0.0, up.power, 0.0)
        assertEquals(0.0, intake.power, 0.0)
        assertEquals(0.0, feeder.power, 0.0)
    }

    @Test
    fun precisionUsesL2WithoutTurningShootingIntoSlowDrive() {
        val precision = CompetitionTeleOp::class.java.getDeclaredMethod("getPrecisionRequested")
            .apply { isAccessible = true }
        driver.raw.right_trigger = 1.0f
        assertFalse(precision.invoke(teleop) as Boolean)
        driver.raw.left_trigger = 1.0f
        assertTrue(precision.invoke(teleop) as Boolean)
    }
}
