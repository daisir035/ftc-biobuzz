package org.firstinspires.ftc.teamcode.opmodes.diagnostics

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.GamepadEx
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.logging.TelemetryBag
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.FakeSink
import org.firstinspires.ftc.teamcode.subsystems.ShovelConfig
import org.firstinspires.ftc.teamcode.subsystems.ShovelSubsystem
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ShovelAngleTestTeleOpTest {
    private val saved = ConfigSnapshot(ShovelConfig)
    private val robot = Robot(FakeHardwareMap())
    private val driver = GamepadEx(Gamepad())
    private val opmode = ShovelAngleTestTeleOp()

    init {
        val bag = TelemetryBag(listOf(FakeSink()), 0.0, Clock { 0L })
        for ((name, value) in listOf("robot" to robot, "driver" to driver, "telemetryBag" to bag)) {
            OpModeBase::class.java.getDeclaredField(name).apply { isAccessible = true }.set(opmode, value)
        }
        ShovelAngleTestTeleOp::class.java.getDeclaredMethod("configure")
            .apply { isAccessible = true }.invoke(opmode)
    }

    @After
    fun tearDown() {
        robot.stop()
        saved.restore()
    }

    private fun tick() {
        driver.update()
        ShovelAngleTestTeleOp::class.java.getDeclaredMethod("onLoop")
            .apply { isAccessible = true }.invoke(opmode)
        Scheduler.execute()
    }

    @Test
    fun linkedPresetChangesOnlyApplyWhileL1IsHeld() {
        ShovelConfig.leftAlignedPosition = 0.36
        ShovelConfig.rightAlignedPosition = 0.225
        ShovelConfig.targetOffsetDegrees = 0.0
        ShovelConfig.preset1OffsetDegrees = 60.0
        val shovel = robot.subsystems().filterIsInstance<ShovelSubsystem>().single()
        driver.lockBindings()
        driver.raw.circle = true
        tick()
        assertFalse(shovel.enabled)
        assertEquals(60.0, ShovelConfig.targetOffsetDegrees, 0.0)

        driver.raw.left_bumper = true
        tick()
        assertTrue(shovel.enabled)
        assertEquals(0.56, shovel.leftPosition, 1e-9)
        assertEquals(0.425, shovel.rightPosition, 1e-9)

        driver.raw.cross = true
        tick()
        assertEquals(0.36, shovel.leftPosition, 1e-9)
        assertEquals(0.225, shovel.rightPosition, 1e-9)
        driver.raw.left_bumper = false
        tick()
        assertFalse(shovel.enabled)
    }
}
