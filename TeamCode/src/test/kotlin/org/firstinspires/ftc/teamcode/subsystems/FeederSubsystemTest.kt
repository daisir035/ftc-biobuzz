package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Scheduler
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.sim.ConfigSnapshot
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.MotorProbe
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test

class FeederSubsystemTest {
    private val savedConfig = ConfigSnapshot(FeederConfig)
    private val motors = RobotConfig.Feeder.MOTORS.map { MotorProbe() }
    private val feeder = FeederSubsystem()

    @Before
    fun setUp() {
        Scheduler.reset()
        val hardwareMap = FakeHardwareMap()
        RobotConfig.Feeder.MOTORS.forEachIndexed { i, name -> hardwareMap.put(name, motors[i].device) }
        feeder.init(hardwareMap)
    }

    @After
    fun tearDown() {
        Scheduler.reset()
        savedConfig.restore()
    }

    private fun tick() {
        Scheduler.execute()
        feeder.writeHardware()
    }

    private fun assertPower(expected: Double) = motors.forEach { assertEquals(expected, it.power, 0.0) }

    @Test
    fun feedingStartsOnceReadyAndDoesNotPauseOnASpeedDip() {
        var ready = false
        val feed = feeder.feedOnceReady(ready = { ready }, spinning = { true })
        Scheduler.schedule(feed)
        tick()
        assertPower(0.0)

        ready = true
        tick()
        assertPower(FeederConfig.feedPower)

        ready = false
        tick()
        assertPower(FeederConfig.feedPower)

        Scheduler.cancel(feed)
        feeder.writeHardware()
        assertPower(0.0)
    }

    @Test
    fun aNewBurstWaitsForReadyAgain() {
        var ready = true
        val feed = feeder.feedOnceReady(ready = { ready }, spinning = { true })
        Scheduler.schedule(feed)
        tick()
        Scheduler.cancel(feed)

        ready = false
        Scheduler.schedule(feed)
        tick()
        assertPower(0.0)
    }

    @Test
    fun switchingTheFlywheelOffStopsTheFeedUntilItIsReadyAgain() {
        var ready = true
        var spinning = true
        Scheduler.schedule(feeder.feedOnceReady(ready = { ready }, spinning = { spinning }))
        tick()
        assertPower(FeederConfig.feedPower)

        spinning = false
        ready = false
        tick()
        assertPower(0.0)

        // Back on but still below speed: the burst does not resume yet.
        spinning = true
        tick()
        assertPower(0.0)

        ready = true
        tick()
        assertPower(FeederConfig.feedPower)
    }

    @Test
    fun faultsAndStopZeroTheMotors() {
        Scheduler.schedule(feeder.feedOnceReady(ready = { true }, spinning = { true }))
        tick()
        feeder.onCommandFault()
        feeder.writeHardware()
        assertPower(0.0)

        Scheduler.schedule(feeder.feedOnceReady(ready = { true }, spinning = { true }))
        tick()
        feeder.stop()
        assertPower(0.0)
    }
}
