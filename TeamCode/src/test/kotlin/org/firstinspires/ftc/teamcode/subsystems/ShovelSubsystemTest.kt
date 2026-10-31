package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoControllerEx
import com.qualcomm.robotcore.hardware.ServoImplEx
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.deviceProxy
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class ShovelSubsystemTest {
    private val positions = mutableMapOf<Int, Double>()
    private val enabled = mutableMapOf<Int, Boolean>()
    private val ranges = mutableMapOf<Int, PwmControl.PwmRange>()
    private val controller = deviceProxy(ServoControllerEx::class.java) { method, args ->
        val port = args.firstOrNull() as? Int
        when (method) {
            "setServoPosition" -> { positions[port!!] = args[1] as Double; null }
            "getServoPosition" -> positions[port] ?: Double.NaN
            "setServoPwmRange" -> { ranges[port!!] = args[1] as PwmControl.PwmRange; null }
            "setServoPwmEnable" -> { enabled[port!!] = true; null }
            "setServoPwmDisable" -> { enabled[port!!] = false; null }
            "isServoPwmEnabled" -> enabled[port] ?: false
            else -> null
        }
    }
    private val shovel = ShovelSubsystem()

    @Before
    fun setUp() {
        Scheduler.reset()
        val hardware = FakeHardwareMap().apply {
            put("left", ServoImplEx(controller, 0, ServoConfigurationType()))
            put("right", ServoImplEx(controller, 1, ServoConfigurationType()))
        }
        shovel.init(hardware)
    }

    @After
    fun tearDown() = Scheduler.reset()

    private fun tick() {
        Scheduler.execute()
        shovel.writeHardware()
    }

    @Test
    fun initAndIdleNeverCommandAnUnknownPosition() {
        shovel.initPeriodic()
        shovel.writeHardware()
        assertTrue(positions.isEmpty())
        assertFalse(enabled.getValue(0))
        assertFalse(enabled.getValue(1))
        assertEquals(PwmControl.PwmRange(500.0, 2500.0), ranges.getValue(0))
        assertEquals(PwmControl.PwmRange(500.0, 2500.0), ranges.getValue(1))
    }

    @Test
    fun applyingPositionsMirrorsTheLeftServoAndReleaseDisablesBothPorts() {
        var target = 0.3
        val command = shovel.holdPositions({ target }, { target })
        Scheduler.schedule(command)
        tick()
        assertEquals(0.7, positions.getValue(0), 1e-9)
        assertEquals(0.3, positions.getValue(1), 1e-9)
        assertTrue(enabled.getValue(0))
        assertTrue(enabled.getValue(1))

        target = 0.4
        tick()
        assertEquals(0.6, positions.getValue(0), 1e-9)
        assertEquals(0.4, positions.getValue(1), 1e-9)
        Scheduler.cancel(command)
        shovel.writeHardware()
        assertFalse(enabled.getValue(0))
        assertFalse(enabled.getValue(1))
    }

    @Test
    fun stopAndFaultDisablePwmWithoutMovingToAnotherPosition() {
        val command = shovel.holdPositions({ 0.2 }, { 0.6 })
        Scheduler.schedule(command)
        tick()
        val before = positions.toMap()
        shovel.onCommandFault()
        assertFalse(enabled.getValue(0))
        assertFalse(enabled.getValue(1))
        tick()
        shovel.stop()
        assertFalse(enabled.getValue(0))
        assertFalse(enabled.getValue(1))
        assertEquals(before, positions)
    }
}
