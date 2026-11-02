package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class ShooterSubsystemTest {
    private val savedConfig = ConfigSnapshot(ShooterConfig)
    private val up = MotorProbe()
    private val down = MotorProbe()
    private val hardware = FakeHardwareMap()
    private var now = 0L
    private var volts = Double.NaN
    private val shooter = ShooterSubsystem(clock = Clock { now }) { volts }

    @Before
    fun setUp() {
        Scheduler.reset()
        hardware.put("up", up.device)
        hardware.put("down", down.device)
        ShooterConfig.kF = 0.0005
        ShooterConfig.kP = 0.001
        ShooterConfig.kI = 0.0
        ShooterConfig.kD = 0.0
        ShooterConfig.ticksPerRev = 103.8
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        savedConfig.restore()
    }

    private fun tick() {
        now += 20_000_000L
        shooter.periodic()
        Scheduler.execute()
        shooter.writeHardware()
    }

    @Test
    fun onlyUpEncoderControlsBothMotorsAtTheSamePower() {
        shooter.init(hardware)
        up.velocity = 1384.0 // 800 RPM at 103.8 ticks/rev; down's encoder is unused.
        down.velocity = 0.0
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()

        assertEquals(800.0, shooter.measuredRpm, 1e-9)
        assertEquals(0.7, up.power, 1e-9)
        assertEquals(0.7, down.power, 1e-9)
        assertEquals(DcMotorSimple.Direction.REVERSE, up.direction)
        assertEquals(DcMotorSimple.Direction.FORWARD, down.direction)
        assertFalse(shooter.atTarget)
    }

    @Test
    fun changingThePresetUpdatesTheRunningCommandAndCancelStopsBothMotors() {
        shooter.init(hardware)
        var rpm = 1000.0
        up.velocity = 1730.0 // 1000 RPM.
        val spin = shooter.spinUp { rpm }
        Scheduler.schedule(spin)
        tick()
        assertTrue(shooter.atTarget)
        assertEquals(1000.0, shooter.targetRpm!!, 0.0)

        rpm = 1200.0
        tick()
        assertEquals(1200.0, shooter.targetRpm!!, 0.0)
        assertFalse(shooter.atTarget)
        assertEquals(0.8, up.power, 1e-9)

        Scheduler.cancel(spin)
        shooter.writeHardware()
        assertEquals(0.0, up.power, 0.0)
        assertEquals(0.0, down.power, 0.0)
        assertFalse(shooter.atTarget)
    }

    @Test
    fun outputIsClampedAndNeverReversesTheFlywheel() {
        shooter.init(hardware)
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        assertEquals(1.0, up.power, 0.0)

        up.velocity = 3460.0 // 2000 RPM, above the target.
        tick()
        assertEquals(0.0, up.power, 0.0)
        assertEquals(0.0, down.power, 0.0)
    }

    @Test
    fun batteryVoltageScalesTheFeedforward() {
        volts = 10.0
        ShooterConfig.kP = 0.0
        shooter.init(hardware)
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        assertEquals(0.6, up.power, 1e-9) // 0.0005 * 1000 * 12 / 10.
    }

    @Test
    fun standaloneTestReadsItsOwnEncoder() {
        val probe = MotorProbe()
        hardware.put("shoot", probe.device)
        val single = ShooterSubsystem(listOf("shoot"), feedbackMotor = "shoot") { Double.NaN }
        single.init(hardware)
        probe.velocity = 1730.0
        single.periodic()
        Scheduler.schedule(single.spinUp { 1000.0 })
        Scheduler.execute()
        single.writeHardware()
        assertEquals(1000.0, single.measuredRpm, 1e-9)
        assertTrue(single.atTarget)
        assertEquals(0.5, probe.power, 1e-9)
    }

    @Test
    fun aNonFiniteSpeedOrTargetWritesZeroAndLeavesNoNanBehind() {
        ShooterConfig.kP = 0.0
        ShooterConfig.kI = 1.0
        shooter.init(hardware)
        ShooterConfig.ticksPerRev = 0.0 // 0 / 0 ticks: measured speed is NaN.
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        tick()
        assertEquals(0.0, up.power, 0.0) // MotorProbe rejects a NaN write outright.

        // Fixed in Panels: the controller restarts clean, feedforward only on its first tick.
        ShooterConfig.ticksPerRev = 103.8
        tick()
        assertEquals(0.5, up.power, 1e-9)

        Scheduler.schedule(shooter.spinUp { Double.NaN })
        tick()
        assertEquals(0.0, up.power, 0.0)
    }

    @Test
    fun eachDipBelowTargetAfterReachingItCountsOneShot() {
        ShooterConfig.speedToleranceRpm = 20.0
        ShooterConfig.shotDipRpm = 60.0
        shooter.init(hardware)
        val spin = shooter.spinUp { 1000.0 }
        Scheduler.schedule(spin)
        tick()
        tick()
        assertEquals(0, shooter.shotsFired) // Spinning up from rest is not a shot.

        up.velocity = 1730.0 // 1000 RPM: at speed, armed.
        tick()
        up.velocity = 1557.0 // 900 RPM: a ball took 100 RPM.
        tick()
        assertEquals(1, shooter.shotsFired)
        tick()
        assertEquals(1, shooter.shotsFired) // Still low: the same shot.

        up.velocity = 1730.0
        tick()
        up.velocity = 1557.0
        tick()
        assertEquals(2, shooter.shotsFired)

        Scheduler.cancel(spin)
        tick()
        tick()
        assertEquals(0, shooter.shotsFired) // Cleared with the target.
    }

    @Test
    fun faultsAndStopZeroTheMotors() {
        shooter.init(hardware)
        val spin = shooter.spinUp { 1000.0 }
        Scheduler.schedule(spin)
        tick()
        shooter.onCommandFault()
        shooter.writeHardware()
        assertEquals(0.0, up.power, 0.0)
        assertEquals(0.0, down.power, 0.0)

        tick()
        assertTrue(up.power > 0.0)
        shooter.stop()
        assertEquals(0.0, up.power, 0.0)
        assertEquals(0.0, down.power, 0.0)
        assertEquals(0.0, shooter.power, 0.0)
    }
}
