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
import org.junit.After
import org.junit.Assert.assertTrue
import org.junit.Test

class ShovelPositionTestTeleOpTest {
    private val saved = ConfigSnapshot(ShovelCalibrationConfig)
    private val robot = Robot(FakeHardwareMap())
    private val driver = GamepadEx(Gamepad())
    private val sink = FakeSink()
    private val bag = TelemetryBag(listOf(sink), 0.0, Clock { 0L })
    private val opmode = ShovelPositionTestTeleOp()

    init {
        for ((name, value) in listOf("robot" to robot, "driver" to driver, "telemetryBag" to bag)) {
            OpModeBase::class.java.getDeclaredField(name).apply { isAccessible = true }.set(opmode, value)
        }
        ShovelPositionTestTeleOp::class.java.getDeclaredMethod("configure")
            .apply { isAccessible = true }.invoke(opmode)
    }

    @After
    fun tearDown() {
        robot.stop()
        saved.restore()
    }

    private fun tick() {
        driver.update()
        ShovelPositionTestTeleOp::class.java.getDeclaredMethod("onLoop")
            .apply { isAccessible = true }.invoke(opmode)
        Scheduler.execute()
        bag.flush()
    }

    @Test
    fun capturesOnlyAppliedTargetsAndKeepsAllThreePairsVisible() {
        ShovelCalibrationConfig.leftPosition = 0.3
        ShovelCalibrationConfig.rightPosition = 0.7
        driver.lockBindings()
        driver.raw.cross = true
        tick()
        assertTrue(sink.data.contains("Raised" to "not captured"))

        driver.raw.cross = false
        driver.raw.left_bumper = true
        tick()
        driver.raw.cross = true
        tick()
        assertTrue(sink.data.contains("Raised" to "left = 0.3000, right = 0.7000"))

        driver.raw.cross = false
        ShovelCalibrationConfig.leftPosition = 0.5
        ShovelCalibrationConfig.rightPosition = 0.9
        tick()
        driver.raw.circle = true
        tick()
        assertTrue(sink.data.contains("Down 60" to "left = 0.5000, right = 0.9000"))

        driver.raw.circle = false
        ShovelCalibrationConfig.leftPosition = 0.6
        ShovelCalibrationConfig.rightPosition = 1.0
        tick()
        driver.raw.square = true
        tick()
        driver.raw.left_bumper = false
        tick()
        sink.reset()
        tick()
        assertTrue(sink.data.contains("Raised" to "left = 0.3000, right = 0.7000"))
        assertTrue(sink.data.contains("Down 60" to "left = 0.5000, right = 0.9000"))
        assertTrue(sink.data.contains("Down 70" to "left = 0.6000, right = 1.0000"))
    }
}
