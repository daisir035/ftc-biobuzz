package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

class ShooterSubsystemTest {
    private val savedConfig = ConfigSnapshot(ShooterConfig)
    private val clock = FakeClock()
    private var volts = 12.0
    private val up = MotorProbe()
    private val down = MotorProbe()
    private val shooter = ShooterSubsystem(clock = clock, batteryVolts = { volts })

    @Before
    fun setUp() {
        Scheduler.reset()
        ShooterConfig.kF = 0.0005
        ShooterConfig.kP = 0.0
        ShooterConfig.kI = 0.0
        ShooterConfig.kD = 0.0
        ShooterConfig.iLimit = 0.2
        ShooterConfig.nominalVolts = 12.0
        ShooterConfig.ticksPerRev = 103.8
        val hardwareMap = FakeHardwareMap()
        hardwareMap.put(RobotConfig.Shooter.UP_MOTOR, up.device)
        hardwareMap.put(RobotConfig.Shooter.DOWN_MOTOR, down.device)
        shooter.init(hardwareMap)
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        savedConfig.restore()
    }

    /** One robot loop: read, commands, write; then 20 ms pass. */
    private fun tick() {
        shooter.periodic()
        Scheduler.execute()
        shooter.writeHardware()
        clock.advanceMs(20.0)
    }

    private fun spinAt(rpm: Double) {
        up.velocity = rpm * ShooterConfig.ticksPerRev / 60.0
    }

    private fun assertPower(expected: Double) {
        assertEquals(expected, up.power, 1e-9)
        assertEquals(expected, down.power, 1e-9)
    }

    @Test
    fun theDownMotorIsReversedSoBothLaunch() {
        assertEquals(DcMotorSimple.Direction.FORWARD, up.direction)
        assertEquals(DcMotorSimple.Direction.REVERSE, down.direction)
    }

    @Test
    fun feedforwardIsScaledByBatteryVoltage() {
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        assertPower(0.5)

        volts = 10.0
        tick()
        assertPower(0.6)
    }

    @Test
    fun measuredSpeedComesFromTheFeedbackEncoder() {
        spinAt(1000.0)
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        assertEquals(1000.0, shooter.measuredRpm, 1e-9)
        assertTrue(shooter.atTarget)

        spinAt(1000.0 - ShooterConfig.speedToleranceRpm - 1.0)
        tick()
        assertFalse(shooter.atTarget)
    }

    @Test
    fun aFlywheelAboveTargetIsNeverBrakedByReversePower() {
        ShooterConfig.kP = 0.01
        spinAt(2000.0)
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        assertPower(0.0)
    }

    @Test
    fun theIntegralTermIsLimitedToILimit() {
        ShooterConfig.kF = 0.0
        ShooterConfig.kI = 1.0
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        repeat(50) { tick() }
        assertPower(ShooterConfig.iLimit)
    }

    @Test
    fun aNonFiniteSpeedWritesZeroAndLeavesNoNanBehind() {
        ShooterConfig.kI = 1.0
        ShooterConfig.ticksPerRev = 0.0 // 0 / 0 ticks: measured speed is NaN
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        tick()
        assertPower(0.0)

        // Fixed in Panels: the controller starts clean (no NaN integral), feedforward only on its first tick.
        ShooterConfig.ticksPerRev = 103.8
        tick()
        assertPower(0.5)
    }

    @Test
    fun aNonFiniteTargetWritesZero() {
        Scheduler.schedule(shooter.spinUp { Double.NaN })
        tick()
        assertPower(0.0)
    }

    @Test
    fun endingTheCommandStopsTheFlywheelAndResetsTheController() {
        ShooterConfig.kI = 1.0
        val spin = shooter.spinUp { 1000.0 }
        Scheduler.schedule(spin)
        repeat(5) { tick() }

        Scheduler.cancel(spin)
        shooter.writeHardware()
        assertNull(shooter.targetRpm)
        assertPower(0.0)

        Scheduler.schedule(spin)
        tick()
        assertPower(0.5)
    }

    @Test
    fun faultsAndStopZeroTheMotors() {
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        shooter.onCommandFault()
        shooter.writeHardware()
        assertPower(0.0)

        Scheduler.schedule(shooter.spinUp { 1000.0 })
        tick()
        shooter.stop()
        assertPower(0.0)
    }
}
