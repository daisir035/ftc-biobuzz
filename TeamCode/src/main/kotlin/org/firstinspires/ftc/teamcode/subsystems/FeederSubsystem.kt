package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged

/**
 * The motor that pushes balls from the intake into the flywheel
 * ([RobotConfig.Feeder.MOTORS]); positive feeds toward the flywheel. No
 * blocker servo and, for now, no reverse. Off unless a command runs it.
 */
class FeederSubsystem : SubsystemBase("Feeder") {

    private var motors: List<DcMotorEx> = emptyList()

    var power: Double = 0.0
        private set

    override fun init(hardwareMap: HardwareMap) {
        motors = RobotConfig.Feeder.MOTORS.map { name ->
            hardwareMap.get(DcMotorEx::class.java, name).apply {
                direction = if (name in REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
    }

    override fun writeHardware() {
        motors.forEach { it.power = power }
    }

    /**
     * Waits until `ready()` is first true (the flywheel at speed), then feeds
     * at [FeederConfig.feedPower] without pausing until interrupted, so a
     * burst is not broken up by the speed dip of each shot.
     */
    fun feedOnceReady(ready: () -> Boolean): Command {
        var released = false
        return logged(
            "Feeder feed",
            Command.build()
                .requiring(this)
                .setStart { released = false }
                .setExecute {
                    if (!released && ready()) released = true
                    power = if (released) FeederConfig.feedPower else 0.0
                }
                .setEnd {
                    released = false
                    power = 0.0
                },
        )
    }

    override fun onCommandFault() {
        power = 0.0
    }

    override fun stop() {
        power = 0.0
        motors.forEach { it.power = 0.0 }
    }

    override fun logState(log: StateLog) {
        log.put("power", power)
    }

    private companion object {
        /**
         * Motor names whose positive power runs balls away from the flywheel.
         * This robot's feeder feeds with FORWARD, so none are reversed.
         */
        val REVERSED = emptySet<String>()
    }
}

/** Feeder power, live-editable in Panels. */
@Configurable
object FeederConfig {
    @JvmField var feedPower = 1.0
}
