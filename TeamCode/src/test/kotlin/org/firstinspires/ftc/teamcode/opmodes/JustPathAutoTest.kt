package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.api.Paths
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.core.Alliance
import org.firstinspires.ftc.teamcode.core.OpModeBase
import org.firstinspires.ftc.teamcode.core.Robot
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.firstinspires.ftc.teamcode.subsystems.IntakeConfig
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveFixture
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class JustPathAutoTest {
    private val saved = listOf(JustPathConfig, AutoConfig).map(::ConfigSnapshot)
    private val hardware = PedroDriveFixture()
    private val robot = Robot(hardware.hardwareMap, hardware.clock)
    private val motor = MotorProbe()
    private val auto = JustPathAutoRed()

    init {
        hardware.hardwareMap.put("intake", motor.device)
        robot.register(hardware.drive)
        val intake = robot.register(IntakeSubsystem())
        intake.init(hardware.hardwareMap)
        AutoBase::class.java.getDeclaredField("drive").apply { isAccessible = true }.set(auto, hardware.drive)
        JustPathAutoRed::class.java.getDeclaredField("intake").apply { isAccessible = true }.set(auto, intake)
        OpModeBase::class.java.getDeclaredField("robot").apply { isAccessible = true }.set(auto, robot)
        robot.start()
    }

    @After fun tearDown() { robot.stop(); saved.forEach { it.restore() } }
    private fun tick(count: Int = 1) = repeat(count) { hardware.clock.advanceMs(20.0); robot.loop() }
    private fun collect(): Command =
        auto.collectAlong("Test FLOWER", Paths.line(Pose.zero(), Pose(48.0, 0.0)).constant(0.0))

    @Test fun theSharedRouteMatchesTheSuppliedEndpointsAndControlPoints() {
        val route = FlowerRoute(Alliance.RED.poses())
        val paths = listOf(route.toFirstFlower(), route.toFarShot(), route.toSecondFlower(), route.toHomeShot())
        val midpoints = listOf(39.0722 to 37.23205, 30.74826875 to 97.50993125,
            55.6097625 to 103.47985, 58.0147 to 100.50405)
        val endpoints = listOf(Pose(23.3387, 42.5526, Math.PI), Pose(66.0339, 115.8782, Math.PI / 2),
            Pose(55.782, 112.4703, Math.PI / 2), Pose(65.18, 116.5975, Math.PI / 2))
        var previous = Pose(57.3401, 7.4974, 3 * Math.PI / 2)
        for (index in 0..3) {
            val path = paths[index]
            assertEquals(previous.x(), path.get(0.0).x(), 1e-6)
            assertEquals(previous.y(), path.get(0.0).y(), 1e-6)
            assertEquals(previous.heading(), path.heading(0.0), 1e-6)
            assertEquals(midpoints[index].first, path.get(0.5).x(), 1e-6)
            assertEquals(midpoints[index].second, path.get(0.5).y(), 1e-6)
            assertEquals(endpoints[index].x(), path.endPose().x(), 1e-6)
            assertEquals(endpoints[index].y(), path.endPose().y(), 1e-6)
            assertEquals(endpoints[index].heading(), path.heading(1.0), 1e-6)
            previous = endpoints[index]
        }
    }

    @Test fun intakeStartsNearFlowerAndStopsAfterCollection() {
        val collection = collect()
        Scheduler.schedule(collection)
        tick(5)
        assertEquals(0.0, motor.power, 0.0)
        hardware.localizer.measuredPose = Pose(24.0, 0.0)
        tick(4)
        assertEquals(0.0, motor.power, 0.0)
        hardware.localizer.measuredPose = Pose(42.0, 0.0)
        tick(4)
        assertEquals(IntakeConfig.collectPower, motor.power, 0.0)
        hardware.localizer.measuredPose = Pose(48.0, 0.0)
        tick(10)
        assertEquals(IntakeConfig.collectPower, motor.power, 0.0)
        tick(40)
        assertFalse(Scheduler.isScheduled(collection))
        assertEquals(0.0, motor.power, 0.0)
    }

    @Test fun cancellationStopsIntakeAndCannotStartAPendingCollection() {
        for (startIntake in listOf(false, true)) {
            hardware.localizer.measuredPose = Pose.zero()
            val collection = collect()
            Scheduler.schedule(collection)
            tick(4)
            if (startIntake) {
                hardware.localizer.measuredPose = Pose(42.0, 0.0)
                tick(4)
                assertEquals(IntakeConfig.collectPower, motor.power, 0.0)
            }
            Scheduler.cancel(collection)
            tick(50)
            assertEquals(0.0, motor.power, 0.0)
        }
    }

    @Test fun routineTimeoutStopsAnActiveIntakeAndDrive() {
        AutoConfig.pathTimeoutMs = 60_000.0
        val path = FlowerRoute(Alliance.RED.poses()).toFirstFlower()
        hardware.localizer.measuredPose = path.get(0.0).let { Pose(it.x(), it.y(), path.heading(0.0)) }
        val routine = auto.buildRoutine()
        Scheduler.schedule(routine)
        tick(5)
        hardware.localizer.measuredPose = path.get(0.9).let { Pose(it.x(), it.y(), path.heading(0.9)) }
        tick(5)
        assertEquals(IntakeConfig.collectPower, motor.power, 0.0)
        tick(1500)
        assertFalse(Scheduler.isScheduled(routine))
        assertEquals(0.0, motor.power, 0.0)
        assertTrue(hardware.powers().all { it == 0.0 })
    }
}
