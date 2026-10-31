package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.FakeClock
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.firstinspires.ftc.teamcode.subsystems.FeederConfig
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeConfig
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test

class CompetitionTeleOpTest {
    private val savedConfigs = listOf(ShooterConfig, IntakeConfig, FeederConfig).map(::ConfigSnapshot)
    private val intakeMotor = MotorProbe()
    private val feederMotor = MotorProbe()
    private val up = MotorProbe()
    private val intake = IntakeSubsystem()
    private val feeder = FeederSubsystem()
    private val shooter = ShooterSubsystem()
    private lateinit var shot: Command

    @Before
    fun setUp() {
        Scheduler.reset()
        ShooterConfig.ticksPerRev = 103.8
        ShooterConfig.targetRpm = 1000.0
        IntakeConfig.collectPower = 1.0
        FeederConfig.feedPower = 1.0
        val hardware = FakeHardwareMap().apply {
            put("intake", intakeMotor.device)
            put("feeder", feederMotor.device)
            put("up", up.device)
            put("down", MotorProbe().device)
        }
        intake.init(hardware)
        feeder.init(hardware)
        shooter.init(hardware)

        // Exercise the real OpMode command without starting Android's run loop or drivetrain.
        val teleop = CompetitionTeleOp()
        for ((name, subsystem) in listOf("intake" to intake, "feeder" to feeder, "shooter" to shooter)) {
            CompetitionTeleOp::class.java.getDeclaredField(name).apply { isAccessible = true }.set(teleop, subsystem)
        }
        shot = CompetitionTeleOp::class.java.getDeclaredMethod("shoot")
            .apply { isAccessible = true }.invoke(teleop) as Command
        Scheduler.schedule(shooter.spinUp())
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        savedConfigs.forEach { it.restore() }
    }

    private fun tick() {
        shooter.periodic()
        Scheduler.execute()
        intake.writeHardware()
        feeder.writeHardware()
        shooter.writeHardware()
    }

    private fun assertFeedPower(expected: Double) {
        assertEquals(expected, intakeMotor.power, 0.0)
        assertEquals(expected, feederMotor.power, 0.0)
    }

    @Test
    fun intakeAndFeederWaitForSpeedThenRunThroughSpeedDipsUntilReleased() {
        Scheduler.schedule(shot)
        repeat(4) { tick() }
        assertFeedPower(0.0)

        up.velocity = 1730.0 // 1000 RPM at 103.8 ticks/rev.
        repeat(4) { tick() }
        assertFeedPower(1.0)

        up.velocity = 0.0
        repeat(4) { tick() }
        assertFeedPower(1.0)

        Scheduler.cancel(shot)
        tick()
        assertFeedPower(0.0)
    }

    @Test
    fun cancelWhileWaitingIsSafeAndEveryNewBurstWaitsAgain() {
        Scheduler.schedule(shot)
        tick()
        Scheduler.cancel(shot)
        tick()
        assertFeedPower(0.0)

        up.velocity = 1730.0
        Scheduler.schedule(shot)
        repeat(4) { tick() }
        assertFeedPower(1.0)
        Scheduler.cancel(shot)

        up.velocity = 0.0
        Scheduler.schedule(shot)
        repeat(4) { tick() }
        assertFeedPower(0.0)
    }

    @Test
    fun shootingCancelsThePendingIntakeReverse() {
        val clock = FakeClock()
        var intaking = true
        val assist = feeder.intakeAssist({ intaking }, clock)
        Scheduler.schedule(assist)
        tick()
        intaking = false
        tick()

        Scheduler.schedule(shot)
        clock.advanceMs(1000.0)
        repeat(4) { tick() }
        assertFeedPower(0.0)

        up.velocity = 1730.0
        repeat(4) { tick() }
        assertFeedPower(1.0)
    }
}
