package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Before
import org.junit.Test

class ScoringTest {
    private val saved = listOf(ShooterConfig, IntakeConfig, FeederConfig, ScoringConfig).map(::ConfigSnapshot)
    private val clock = FakeClock()
    private val intakeMotor = MotorProbe()
    private val feederMotor = MotorProbe()
    private val up = MotorProbe()
    private val down = MotorProbe()
    private val intake = IntakeSubsystem()
    private val feeder = FeederSubsystem()
    private val shooter = ShooterSubsystem(clock = clock)
    private val events = mutableListOf<String>()
    private val scoring = Scoring(intake, feeder, shooter, clock) { events += it }

    @Before
    fun setUp() {
        Scheduler.reset()
        ShooterConfig.ticksPerRev = 103.8
        ShooterConfig.kF = 0.0005
        ShooterConfig.kP = 0.001
        ShooterConfig.kI = 0.0
        ShooterConfig.kD = 0.0
        ShooterConfig.speedToleranceRpm = 20.0
        ShooterConfig.shotDipRpm = 60.0
        IntakeConfig.collectPower = 1.0
        FeederConfig.feedPower = 1.0
        FeederConfig.intakePower = 0.2
        FeederConfig.intakeReleaseDelayMs = 200.0
        FeederConfig.intakeReversePower = -0.2
        FeederConfig.intakeReverseMs = 150.0
        val hardware = FakeHardwareMap().apply {
            put("intake", intakeMotor.device)
            put("feeder", feederMotor.device)
            put("up", up.device)
            put("down", down.device)
        }
        intake.init(hardware)
        feeder.init(hardware)
        shooter.init(hardware)
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        saved.forEach { it.restore() }
    }

    private fun tick(count: Int = 1) = repeat(count) {
        clock.advanceMs(20.0)
        shooter.periodic()
        Scheduler.execute()
        intake.writeHardware()
        feeder.writeHardware()
        shooter.writeHardware()
    }

    private fun until(condition: () -> Boolean) {
        repeat(200) { if (condition()) return; tick() }
        fail("condition never became true")
    }

    private fun spinAt(rpm: Double) {
        up.velocity = rpm * 103.8 / 60.0
    }

    private fun assertFeeding(expected: Double) {
        assertEquals(expected, intakeMotor.power, 0.0)
        assertEquals(expected, feederMotor.power, 0.0)
    }

    // ------------------------------------------------------------------ collect

    @Test
    fun collectRunsTheFeederWithTheIntakeThenReversesBrieflyAfterRelease() {
        var active = true
        val collect = scoring.collect { active }
        Scheduler.schedule(collect)
        tick()
        assertEquals(IntakeConfig.collectPower, intakeMotor.power, 0.0)
        assertEquals(FeederConfig.intakePower, feederMotor.power, 0.0)

        active = false
        tick()
        assertEquals(0.0, intakeMotor.power, 0.0)
        assertEquals(0.0, feederMotor.power, 0.0)

        tick(10) // 200 ms after release.
        assertEquals(FeederConfig.intakeReversePower, feederMotor.power, 0.0)
        assertEquals(0.0, intakeMotor.power, 0.0)

        tick(8)
        assertFalse(Scheduler.isScheduled(collect))
        assertEquals(0.0, feederMotor.power, 0.0)
    }

    @Test
    fun collectingAgainBeforeTheReverseResumesBoth() {
        var active = true
        Scheduler.schedule(scoring.collect { active })
        tick()
        active = false
        tick(3)
        active = true
        tick()
        assertEquals(IntakeConfig.collectPower, intakeMotor.power, 0.0)
        assertEquals(FeederConfig.intakePower, feederMotor.power, 0.0)
    }

    @Test
    fun aShotTakesOverACollectAndSkipsItsReverse() {
        var active = true
        Scheduler.schedule(scoring.collect { active })
        tick()
        spinAt(1000.0)
        Scheduler.schedule(scoring.shoot { 1000.0 })
        active = false
        tick(30)
        assertTrue(feederMotor.writes.none { it < 0.0 })
        assertFeeding(1.0)
    }

    // -------------------------------------------------------------- teleop shot

    @Test
    fun shootWaitsForSpeedThenFeedsThroughDipsUntilReleased() {
        val shot = scoring.shoot { 1000.0 }
        Scheduler.schedule(shot)
        tick(4)
        assertEquals(1000.0, shooter.targetRpm!!, 0.0)
        assertFeeding(0.0)

        spinAt(1000.0)
        until { feederMotor.power > 0.0 }
        assertFeeding(1.0)

        spinAt(0.0)
        tick(4)
        assertFeeding(1.0)

        Scheduler.cancel(shot)
        tick()
        assertFeeding(0.0)
        assertNull(shooter.targetRpm)
        assertEquals(0.0, up.power, 0.0)
    }

    @Test
    fun everyNewShotWaitsForSpeedAgain() {
        val shot = scoring.shoot { 1000.0 }
        spinAt(1000.0)
        Scheduler.schedule(shot)
        until { feederMotor.power > 0.0 }
        Scheduler.cancel(shot)
        tick()

        spinAt(0.0)
        Scheduler.schedule(shot)
        tick(5)
        assertFeeding(0.0)
    }

    // ---------------------------------------------------------------- auto burst

    private fun scheduleBurst(ready: () -> Boolean): Command {
        val burst = scoring.burst("Test", ready)
        Scheduler.schedule(shooter.spinUp { 1000.0 })
        Scheduler.schedule(burst)
        return burst
    }

    @Test
    fun aBurstEndsSoonAfterTheExpectedShots() {
        ScoringConfig.ballsPerBurst = 2
        ScoringConfig.shotTailMs = 100.0
        ScoringConfig.burstMaxMs = 10_000.0
        spinAt(1000.0)
        val burst = scheduleBurst { true }
        until { feederMotor.power == FeederConfig.feedPower }

        repeat(2) {
            spinAt(900.0)
            tick()
            spinAt(1000.0)
            tick()
        }
        assertEquals(2, shooter.shotsFired)
        assertTrue(Scheduler.isScheduled(burst)) // The tail still runs.
        tick(7)
        assertFalse(Scheduler.isScheduled(burst))
        assertFeeding(0.0)
        assertEquals(1000.0, shooter.targetRpm!!, 0.0) // The caller's spin-up is untouched.
    }

    @Test
    fun aBurstWithUndetectedShotsEndsAtItsTimeLimit() {
        ScoringConfig.burstMaxMs = 200.0
        spinAt(1000.0)
        val burst = scheduleBurst { true }
        until { feederMotor.power > 0.0 }
        tick(12)
        assertFalse(Scheduler.isScheduled(burst))
        assertFeeding(0.0)
    }

    @Test
    fun anUnreadyBurstFeedsAfterTheTimeoutButNeverBelowSpeed() {
        ScoringConfig.readyTimeoutMs = 100.0
        ScoringConfig.burstMaxMs = 10_000.0
        spinAt(0.0)
        scheduleBurst { false }
        tick(15)
        assertTrue(events.any { "fed before ready" in it })
        assertFeeding(0.0) // Window open, flywheel still slow: neither pushes a ball.

        spinAt(1000.0)
        until { feederMotor.power == FeederConfig.feedPower }
    }
}
