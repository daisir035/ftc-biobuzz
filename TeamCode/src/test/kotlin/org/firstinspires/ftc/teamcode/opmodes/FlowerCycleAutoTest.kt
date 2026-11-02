package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.PoseFactory
import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.firstinspires.ftc.teamcode.subsystems.FeederConfig
import org.firstinspires.ftc.teamcode.subsystems.FeederSubsystem
import org.firstinspires.ftc.teamcode.subsystems.IntakeConfig
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveFixture
import org.firstinspires.ftc.teamcode.subsystems.Scoring
import org.firstinspires.ftc.teamcode.subsystems.ScoringConfig
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
    private val saved = listOf(FlowerCycleConfig, AutoConfig, ScoringConfig, ShooterConfig, FeederConfig, IntakeConfig)
        .map(::ConfigSnapshot)
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

    /** A small square course, independent of the real route. */
    private class TestRoute : FlowerRoute(PoseFactory.degrees()) {
        override val start get() = Pose(0.0, 0.0, 0.0)
        override val firstFlower get() = Pose(24.0, 0.0, 0.0)
        override val farShot get() = Pose(24.0, 24.0, 0.0)
        override val secondFlower get() = Pose(0.0, 24.0, 0.0)
        override val homeShot get() = start
        override fun toFirstFlower() = Paths.line(start, firstFlower).linear(start, firstFlower)
        override fun toFarShot() = Paths.line(firstFlower, farShot).linear(firstFlower, farShot)
        override fun toSecondFlower() = Paths.line(farShot, secondFlower).linear(farShot, secondFlower)
        override fun toHomeShot() = Paths.line(secondFlower, homeShot).linear(secondFlower, homeShot)
    }

    private class TestAuto : FlowerCycleAutoRed() {
        override val route: FlowerRoute = TestRoute()
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
        ShooterConfig.speedToleranceRpm = 20.0
        FeederConfig.intakePower = 0.2
        FeederConfig.feedPower = 1.0
        ScoringConfig.readyTimeoutMs = 300.0
        ScoringConfig.burstMaxMs = 100.0
        FlowerCycleConfig.firstCollectMs = 100.0
        FlowerCycleConfig.secondCollectMs = 100.0
        FlowerCycleConfig.settleTimeoutMs = 100.0
        FlowerCycleConfig.spinUpProgress = 0.7
        FlowerCycleConfig.intakeStartDistanceIn = 6.0
        AutoConfig.pathTimeoutMs = 2000.0
        hardware.hardwareMap.put("up", up.device)
        hardware.hardwareMap.put("down", down.device)
        hardware.hardwareMap.put("intake", intakeMotor.device)
        hardware.hardwareMap.put("feeder", feederMotor.device)
        robot.register(hardware.drive)
        for (subsystem in listOf(shooter, intake, feeder)) {
            robot.register(subsystem)
            subsystem.init(hardware.hardwareMap)
        }
        AutoBase::class.java.getDeclaredField("drive").apply { isAccessible = true }.set(auto, hardware.drive)
        inject("shooter", shooter)
        inject("intake", intake)
        inject("feeder", feeder)
        inject("scoring", Scoring(intake, feeder, shooter, hardware.clock))
        OpModeBase::class.java.getDeclaredField("robot").apply { isAccessible = true }.set(auto, robot)
        robot.start()
    }

    private fun inject(name: String, value: Any) =
        FlowerCycleAutoRed::class.java.getDeclaredField(name).apply { isAccessible = true }.set(auto, value)

    /** A private step factory of [FlowerCycleAutoRed]: shootAt(name, target, rpm, approach) or collectAt(name, path, flower, dwellMs, lowerDelayMs). */
    private fun step(name: String, vararg args: Any?): Command =
        FlowerCycleAutoRed::class.java.declaredMethods.single { it.name == name }
            .apply { isAccessible = true }.invoke(auto, *args) as Command

    private fun tick(count: Int = 1) = repeat(count) {
        hardware.clock.advanceMs(20.0)
        robot.loop()
    }

    private fun until(condition: () -> Boolean) {
        repeat(400) { if (condition()) return; tick() }
        fail("Routine did not reach expected phase")
    }

    private fun measuredRpm(rpm: Double) { up.velocity = rpm * 103.8 / 60.0 }
    private fun shovelLowered() = FlowerCycleAutoRed::class.java.getDeclaredField("shovelLowered")
        .apply { isAccessible = true }.getBoolean(auto)
    private fun place(x: Double, y: Double) { hardware.localizer.measuredPose = Pose(x, y, 0.0) }
    private fun assertStopped() {
        assertNull(shooter.targetRpm)
        assertEquals(0.0, up.power, 0.0)
        assertEquals(0.0, down.power, 0.0)
        assertEquals(0.0, intakeMotor.power, 0.0)
        assertEquals(0.0, feederMotor.power, 0.0)
    }

    @After fun tearDown() { robot.stop(); saved.forEach { it.restore() } }

    // --------------------------------------------------------------- shot step

    @Test fun approachStartsFlywheelAtMarkerAndKeepsItRunningThroughTheShot() {
        val target = Pose(48.0, 0.0, 0.0)
        val path = Paths.line(Pose.zero(), target).constant(0.0)
        val shot = step("shootAt", "Test shot", target, { 1000.0 }, path)
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

    @Test fun aShotNeverFeedsBelowSpeedAndStillEndsWithinItsLimits() {
        measuredRpm(0.0)
        val shot = step("shootAt", "Slow shot", Pose.zero(), { 1000.0 }, null)
        Scheduler.schedule(shot)
        until { !Scheduler.isScheduled(shot) }
        tick()
        assertTrue(feederMotor.writes.all { it == 0.0 })
        assertTrue(intakeMotor.writes.all { it == 0.0 })
        assertStopped()
    }

    @Test fun aShotAwayFromItsPoseFeedsOnceTheReadyWaitRunsOut() {
        // Blocked short of the shot pose: carrying balls to the next FLOWER is worse than a late shot.
        measuredRpm(1000.0)
        val shot = step("shootAt", "Blocked shot", Pose(12.0, 0.0, 0.0), { 1000.0 }, null)
        Scheduler.schedule(shot)
        tick(10)
        assertEquals(0.0, feederMotor.power, 0.0)
        until { feederMotor.power == FeederConfig.feedPower }
        until { !Scheduler.isScheduled(shot) }
        tick()
        assertStopped()
    }

    @Test fun cancellationBeforeMarkerDoesNotStartAnUnstartedFlywheel() {
        val target = Pose(48.0, 0.0, 0.0)
        val shot = step("shootAt", "Cancelled shot", target, { 1000.0 }, Paths.line(Pose.zero(), target).constant(0.0))
        Scheduler.schedule(shot)
        tick(5)
        Scheduler.cancel(shot)
        tick(5)
        assertStopped()
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun shortFinalApproachRaisesShovelsOnArrivalBeforeShooting() {
        inject("shovelLowered", true)
        val target = Pose(24.0, 0.0)
        val shot = step("shootAt", "Home shot", target, { 1000.0 }, Paths.line(Pose.zero(), target).constant(0.0))
        Scheduler.schedule(shot)
        until { hardware.drive.isFollowing }
        assertTrue(shovelLowered())
        place(24.0, 0.0)
        measuredRpm(1000.0)
        until { feederMotor.power > 0.0 }
        assertFalse(shovelLowered())
    }

    @Test fun shortApproachStillSpinsUpWhenItsMarkerWasNotSampled() {
        FlowerCycleConfig.spinUpProgress = 1.0
        val target = Pose(24.0, 0.0, 0.0)
        val shot = step("shootAt", "Short shot", target, { 1000.0 }, Paths.line(Pose.zero(), target).constant(0.0))
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

    // ------------------------------------------------------------ collect step

    @Test fun intakeAndFeederStartOnlyNearTheFlower() {
        val flower = Pose(48.0, 0.0, 0.0)
        val collection = step("collectAt", "Test FLOWER", Paths.line(Pose.zero(), flower).constant(0.0), flower, 100.0, 0.0)
        Scheduler.schedule(collection)
        tick(5)
        assertTrue(hardware.drive.isFollowing)
        assertEquals(0.0, intakeMotor.power, 0.0)
        assertEquals(0.0, feederMotor.power, 0.0)
        place(30.0, 0.0); tick(4)
        assertEquals(0.0, intakeMotor.power, 0.0) // 18 in out: still off.

        place(44.0, 0.0)
        until { intakeMotor.power == IntakeConfig.collectPower }
        assertEquals(FeederConfig.intakePower, feederMotor.power, 0.0)
        place(48.0, 0.0)
        until { !Scheduler.isScheduled(collection) }
        tick()
        assertEquals(0.0, intakeMotor.power, 0.0)
        assertEquals(0.0, feederMotor.power, 0.0)
    }

    @Test fun collectionWaitsForWallClearanceAndCancelledMarkersCannotLowerShovels() {
        val flower = Pose(60.0, 60.0)
        val path = Paths.line(Pose(60.0, 133.0), flower).constant(0.0)
        place(60.0, 133.0)
        val collection = step("collectAt", "Wall FLOWER", path, flower, 100.0, 0.0)
        Scheduler.schedule(collection)
        tick(60)
        assertFalse(shovelLowered())
        place(60.0, 124.0)
        tick(4)
        assertFalse(shovelLowered())
        place(60.0, 123.5)
        tick(4)
        assertTrue(shovelLowered())
        Scheduler.cancel(collection)

        inject("shovelLowered", false)
        place(60.0, 133.0)
        val cancelled = step("collectAt", "Cancelled FLOWER", path, flower, 100.0, 1000.0)
        Scheduler.schedule(cancelled)
        tick(5)
        Scheduler.cancel(cancelled)
        place(60.0, 100.0)
        tick(60)
        assertFalse(shovelLowered())
    }

    // ------------------------------------------------------------ whole routine

    @Test fun wholeRoutineShootsThreeTimesAndCollectsTwiceInOrder() {
        measuredRpm(1000.0)
        val routine = auto.buildRoutine()
        Scheduler.schedule(routine)

        // Preload, from the start pose, at the near preset.
        until { feederMotor.power == FeederConfig.feedPower }
        assertEquals(1000.0, shooter.targetRpm!!, 0.0)

        // First FLOWER: off while far away, on near it.
        until { hardware.drive.isFollowing && feederMotor.power == 0.0 }
        tick(5)
        assertEquals(0.0, intakeMotor.power, 0.0)
        place(20.0, 0.0)
        until { intakeMotor.power == IntakeConfig.collectPower }
        assertEquals(FeederConfig.intakePower, feederMotor.power, 0.0)
        place(24.0, 0.0)

        // Far shot: the flywheel starts late in the approach.
        until { hardware.drive.isFollowing && intakeMotor.power == 0.0 }
        assertNull(shooter.targetRpm)
        measuredRpm(1400.0)
        place(24.0, 20.0)
        until { shooter.targetRpm == 1400.0 }
        place(24.0, 24.0)
        until { feederMotor.power == FeederConfig.feedPower }

        // Second FLOWER.
        until { hardware.drive.isFollowing && feederMotor.power == 0.0 }
        assertEquals(0.0, intakeMotor.power, 0.0)
        place(3.0, 24.0)
        until { intakeMotor.power == IntakeConfig.collectPower }
        place(0.0, 24.0)

        // Home shot at the far preset.
        until { hardware.drive.isFollowing && intakeMotor.power == 0.0 }
        place(0.0, 4.0)
        until { shooter.targetRpm == 1400.0 }
        place(0.0, 0.0)
        until { feederMotor.power == FeederConfig.feedPower }
        assertFalse(shovelLowered())

        until { !Scheduler.isScheduled(routine) }
        tick()
        assertStopped()
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun overallTimeoutStopsAStalledRoutine() {
        AutoConfig.pathTimeoutMs = 60_000.0
        val routine = auto.buildRoutine()
        Scheduler.schedule(routine)
        tick(1500)
        assertFalse(Scheduler.isScheduled(routine))
        assertStopped()
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun pathTimeoutBeforeArrivalStopsTheRoutineInsteadOfDrivingTheNextPath() {
        AutoConfig.pathTimeoutMs = 200.0
        measuredRpm(1000.0)
        val routine = auto.buildRoutine()
        Scheduler.schedule(routine)
        until { hardware.drive.isFollowing }
        // Stay at the start, as if the robot is blocked by a wall.
        tick(50)
        assertFalse(Scheduler.isScheduled(routine))
        assertStopped()
        assertEquals("IDLE", hardware.drive.driveModeName)
        assertTrue(hardware.powers().all { it == 0.0 })
        tick(50)
        assertTrue(hardware.powers().all { it == 0.0 })
    }

    @Test fun cancellingAfterSettlingStopsResidualHoldingImmediately() {
        measuredRpm(1000.0)
        val routine = auto.buildRoutine()
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

    @Test fun unconfiguredPathsCannotBuildARunnableRoutine() {
        assertThrows(IllegalStateException::class.java) { UnconfiguredAuto().buildRoutine() }
        assertStopped()
    }

    // --------------------------------------------------------------- real route

    @Test fun realRouteInterpolatesHeadingDuringTheFirstTwoPaths() {
        val route = FlowerRoute(Alliance.RED.poses())
        for ((path, headings) in listOf<Pair<Path, DoubleArray>>(
            route.toFirstFlower() to doubleArrayOf(270.0, 180.0),
            route.toFarShot() to doubleArrayOf(180.0, 90.0),
            route.toSecondFlower() to doubleArrayOf(90.0, 90.0),
            route.toHomeShot() to doubleArrayOf(90.0, 90.0),
        )) {
            val start = Math.toRadians(headings[0])
            val end = Math.toRadians(headings[1])
            assertEquals(start, path.heading(0.0), 1e-6)
            assertEquals(end, path.heading(1.0), 1e-6)
            if (start != end) {
                // Curve parameter t is not distance fraction; heading is linear in distance.
                assertTrue(path.heading(0.5) < start && path.heading(0.5) > end)
            } else {
                assertEquals(start, path.heading(0.5), 1e-6)
            }
        }
    }

    @Test fun realFirstPathProducesTurningPowerBeforeItsEndpoint() {
        val path = FlowerRoute(Alliance.RED.poses()).toFirstFlower()
        hardware.localizer.measuredPose = path.get(0.0)
        Scheduler.schedule(hardware.drive.followCommand(path))
        tick(3)
        val midpoint = path.get(0.5)
        // Simulate driving halfway along the curve without turning yet.
        hardware.localizer.measuredPose = Pose(midpoint.x(), midpoint.y(), path.heading(0.0))
        tick(3)
        assertTrue(hardware.drive.isFollowing)
        val powers = hardware.powers()
        val turningPower = (-powers[0] + powers[1] - powers[2] + powers[3]) / 4.0
        assertTrue("Follower should command a clockwise turn during travel", turningPower < -0.01)
    }
}
