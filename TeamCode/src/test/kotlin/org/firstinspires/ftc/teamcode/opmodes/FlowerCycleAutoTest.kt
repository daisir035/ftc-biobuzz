package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.firstinspires.ftc.teamcode.subsystems.FeederConfig
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeConfig
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveFixture
import org.firstinspires.ftc.teamcode.subsystems.ShooterConfig
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Assert.fail
import org.junit.Test

class FlowerCycleAutoTest {
    private val saved = listOf(FlowerCycleConfig, ShooterConfig, FeederConfig, IntakeConfig).map(::ConfigSnapshot)
    private val hardware = PedroDriveFixture()
    private val robot = Robot(hardware.hardwareMap, hardware.clock)
    private val up = MotorProbe()
    private val down = MotorProbe()
    private val intakeMotor = MotorProbe()
    private val feederMotor = MotorProbe()
    private val shooter = ShooterSubsystem(clock = hardware.clock)
    private val intake = IntakeSubsystem()
    private val feeder = FeederSubsystem()
    private val auto = TestAuto()

    private class TestAuto : FlowerCycleAutoRed() {
        override val pathsConfigured = true
        override val start get() = Pose(0.0, 0.0, 0.0)
        override val firstFlower get() = Pose(24.0, 0.0, 0.0)
        override val farShot get() = Pose(24.0, 24.0, 0.0)
        override val secondFlower get() = Pose(0.0, 24.0, 0.0)
        override val homeShot get() = start
        // Exercise the routine on a small synthetic course, independently of the user's route.
        override fun toFirstFlower() = Paths.line(start, firstFlower).linear(start, firstFlower)
        override fun toFarShot() = Paths.line(firstFlower, farShot).linear(firstFlower, farShot)
        override fun toSecondFlower() = Paths.line(farShot, secondFlower).linear(farShot, secondFlower)
        override fun toHomeShot() = Paths.line(secondFlower, homeShot).linear(secondFlower, homeShot)
    }

    private class UnconfiguredAuto : FlowerCycleAutoRed() {
        override val pathsConfigured = false
    }

    init {
        ShooterConfig.nearRpm = 1000.0
        ShooterConfig.farRpm = 1400.0
        ShooterConfig.ticksPerRev = 103.8
        ShooterConfig.kF = 0.0005
        ShooterConfig.kP = 0.001
        ShooterConfig.kI = 0.0
        ShooterConfig.kD = 0.0
        FlowerCycleConfig.preloadFeedMs = 100.0
        FlowerCycleConfig.farFeedMs = 100.0
        FlowerCycleConfig.homeFeedMs = 100.0
        FlowerCycleConfig.firstCollectMs = 100.0
        FlowerCycleConfig.secondCollectMs = 100.0
        FlowerCycleConfig.shovelSettleMs = 100.0
        FlowerCycleConfig.readyTimeoutMs = 300.0
        FlowerCycleConfig.settleTimeoutMs = 100.0
        FlowerCycleConfig.pathTimeoutMs = 2000.0
        FlowerCycleConfig.spinUpProgress = 0.7
        hardware.hardwareMap.put("up", up.device)
        hardware.hardwareMap.put("down", down.device)
        hardware.hardwareMap.put("intake", intakeMotor.device)
        hardware.hardwareMap.put("feeder", feederMotor.device)
        robot.register(hardware.drive)
        for (subsystem in listOf(shooter, intake, feeder)) {
            robot.register(subsystem)
            subsystem.init(hardware.hardwareMap)
        }
        inject("drive", hardware.drive)
        inject("shooter", shooter)
        inject("intake", intake)
        inject("feeder", feeder)
        OpModeBase::class.java.getDeclaredField("robot").apply { isAccessible = true }.set(auto, robot)
        robot.start()
    }

    private fun inject(name: String, value: Any) =
        FlowerCycleAutoRed::class.java.getDeclaredField(name).apply { isAccessible = true }.set(auto, value)

    private fun command(name: String, vararg args: Any?): Command =
        FlowerCycleAutoRed::class.java.declaredMethods.single { it.name == name }
            .apply { isAccessible = true }.invoke(auto, *args) as Command

    private fun tick(count: Int = 1) = repeat(count) {
        hardware.clock.advanceMs(20.0)
        robot.loop()
    }

    private fun until(condition: () -> Boolean) {
        repeat(300) { if (condition()) return; tick() }
        fail("Routine did not reach expected phase")
    }

    private fun measuredRpm(rpm: Double) { up.velocity = rpm * 103.8 / 60.0 }
    private fun place(x: Double, y: Double) { hardware.localizer.measuredPose = Pose(x, y, 0.0) }
    private fun assertStopped() {
        assertNull(shooter.targetRpm)
        assertEquals(0.0, up.power, 0.0)
        assertEquals(0.0, down.power, 0.0)
        assertEquals(0.0, intakeMotor.power, 0.0)
        assertEquals(0.0, feederMotor.power, 0.0)
    }

    @After fun tearDown() { robot.stop(); saved.forEach { it.restore() } }

    @Test fun approachStartsFlywheelAtMarkerAndKeepsItRunningThroughTheShot() {
        val target = Pose(48.0, 0.0, 0.0)
        val path = Paths.line(Pose.zero(), target).constant(0.0)
        val shot = command("shootAt", "Test shot", target, { 1000.0 }, 100.0, path)
        Scheduler.schedule(shot)
        tick(8)
        assertNull(shooter.targetRpm)
        place(24.0, 0.0); tick(4)
        assertNull(shooter.targetRpm)
        place(38.0, 0.0)
        until { shooter.targetRpm != null }
        assertTrue(hardware.drive.isFollowing)
        measuredRpm(1000.0)
        tick(3)
        assertEquals(0.0, feederMotor.power, 0.0)
        place(48.0, 0.0)
        until { feederMotor.power > 0.0 }
        assertEquals(1000.0, shooter.targetRpm!!, 0.0)
        until { !Scheduler.isScheduled(shot) }
        tick()
        assertStopped()
    }

    @Test fun missingSpeedOrArrivalSkipsFeedingAndEndsTheShot() {
        for (speedReady in listOf(false, true)) {
            place(0.0, 0.0)
            measuredRpm(if (speedReady) 1000.0 else 0.0)
            val target = if (speedReady) Pose(12.0, 0.0, 0.0) else Pose.zero()
            val shot = command("shootAt", "Unready shot", target, { 1000.0 }, 100.0, null)
            Scheduler.schedule(shot)
            until { !Scheduler.isScheduled(shot) }
            tick()
            assertTrue(feederMotor.writes.all { it == 0.0 })
            assertStopped()
        }
    }

    @Test fun cancellationBeforeMarkerDoesNotStartAnUnstartedFlywheel() {
        val target = Pose(48.0, 0.0, 0.0)
        val shot = command("shootAt", "Cancelled shot", target, { 1000.0 }, 100.0,
            Paths.line(Pose.zero(), target).constant(0.0))
        Scheduler.schedule(shot)
        tick(5)
        Scheduler.cancel(shot)
        tick(5)
        assertStopped()
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun wholeRoutineShootsThreeTimesAndCollectsTwiceInOrder() {
        measuredRpm(1000.0)
        val routine = command("buildRoutine")
        Scheduler.schedule(routine)
        until { feederMotor.power == FeederConfig.feedPower }
        until { hardware.drive.isFollowing && intakeMotor.power == IntakeConfig.collectPower }
        assertEquals(IntakeConfig.collectPower, intakeMotor.power, 0.0)
        assertEquals(FeederConfig.intakePower, feederMotor.power, 0.0)
        assertTrue(FlowerCycleAutoRed::class.java.getDeclaredField("shovelLowered")
            .apply { isAccessible = true }.getBoolean(auto))
        place(24.0, 0.0)
        until { !hardware.drive.isFollowing }
        until { hardware.drive.isFollowing }
        measuredRpm(1400.0)
        place(24.0, 24.0)
        until { feederMotor.power == FeederConfig.feedPower }
        assertEquals(1400.0, shooter.targetRpm!!, 0.0)
        until { hardware.drive.isFollowing && feederMotor.power == FeederConfig.intakePower }
        assertEquals(FeederConfig.intakePower, feederMotor.power, 0.0)
        place(0.0, 24.0)
        until { !hardware.drive.isFollowing }
        until { hardware.drive.isFollowing }
        measuredRpm(1400.0)
        place(0.0, 0.0)
        until { feederMotor.power == FeederConfig.feedPower }
        assertEquals(1400.0, shooter.targetRpm!!, 0.0)
        until { !Scheduler.isScheduled(routine) }
        tick()
        assertStopped()
    }

    @Test fun overallTimeoutStopsAStalledRoutine() {
        FlowerCycleConfig.pathTimeoutMs = 60000.0
        val routine = command("buildRoutine")
        Scheduler.schedule(routine)
        tick(1500)
        assertFalse(Scheduler.isScheduled(routine))
        assertStopped()
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun cancellingAfterSettlingStopsResidualHoldingImmediately() {
        measuredRpm(1000.0)
        val routine = command("buildRoutine")
        Scheduler.schedule(routine)
        until { feederMotor.power > 0.0 }
        place(0.0, 3.0)
        tick()
        assertEquals("HOLDING", hardware.drive.driveModeName)
        assertTrue(hardware.powers().any { it != 0.0 })
        Scheduler.cancel(routine)
        // Cleanup must stop the follower now, before another loop/SDK stop request.
        assertEquals("IDLE", hardware.drive.driveModeName)
        assertTrue(hardware.powers().all { it == 0.0 })
        tick(3)
        assertStopped()
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun shortApproachStillSpinsUpWhenItsMarkerWasNotSampled() {
        FlowerCycleConfig.spinUpProgress = 1.0
        val target = Pose(24.0, 0.0, 0.0)
        val shot = command("shootAt", "Short shot", target, { 1000.0 }, 100.0,
            Paths.line(Pose.zero(), target).constant(0.0))
        Scheduler.schedule(shot)
        until { hardware.drive.isFollowing }
        place(24.0, 0.0)
        measuredRpm(1000.0)
        until { feederMotor.power > 0.0 }
        assertEquals(1000.0, shooter.targetRpm!!, 0.0)
        Scheduler.cancel(shot)
        tick(5)
        assertStopped()
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun unconfiguredPathsCannotBuildARunnableRoutine() {
        val unconfigured = UnconfiguredAuto()
        val error = assertThrows(java.lang.reflect.InvocationTargetException::class.java) {
            FlowerCycleAutoRed::class.java.getDeclaredMethod("buildRoutine")
                .apply { isAccessible = true }.invoke(unconfigured)
        }
        assertTrue(error.cause is IllegalStateException)
        assertStopped()
    }
}
