package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged

/**
 * The motor that pushes balls from the intake into the flywheel
 * ([RobotConfig.Feeder.MOTORS]); positive feeds toward the flywheel. No
 * blocker servo. Off unless a command runs it.
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

    /** Runs backwards until interrupted to eject balls through the intake. */
    fun eject(): Command = logged(
        "Feeder eject",
        infinite { power = FeederConfig.ejectPower }.requiring(this).setEnd { power = 0.0 },
    )

    /** Low-power assist while intaking, then a delayed reverse pulse after release. */
    fun intakeAssist(intaking: () -> Boolean, clock: Clock = Clock.SYSTEM): Command {
        var releasedAtNs: Long? = null
        return logged(
            "Feeder intake assist",
            Command.build()
                .requiring(this)
                .setStart { releasedAtNs = null }
                .setExecute {
                    if (intaking()) {
                        releasedAtNs = null
                        power = FeederConfig.intakePower
                    } else {
                        val now = clock.nanos()
                        val releasedAt = releasedAtNs ?: now.also { releasedAtNs = it }
                        val elapsedMs = (now - releasedAt) / 1e6
                        power = if (elapsedMs >= FeederConfig.intakeReleaseDelayMs &&
                            elapsedMs < FeederConfig.intakeReleaseDelayMs + FeederConfig.intakeReverseMs
                        ) FeederConfig.intakeReversePower else 0.0
                    }
                }
                .setDone {
                    releasedAtNs?.let {
                        (clock.nanos() - it) / 1e6 >= FeederConfig.intakeReleaseDelayMs + FeederConfig.intakeReverseMs
                    } ?: false
                }
                .setEnd { power = 0.0 },
        )
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
    @JvmField var ejectPower = -1.0
    @JvmField var feedPower = 1.0
    @JvmField var intakePower = 1.0
    @JvmField var intakeReleaseDelayMs = 200.0
    @JvmField var intakeReversePower = -0.2
    @JvmField var intakeReverseMs = 150.0
}
