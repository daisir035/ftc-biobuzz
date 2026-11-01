package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.Clock
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged

/**
 * Flywheel motors ([RobotConfig.Shooter.MOTORS] by default), mechanically
 * synchronized, so one encoder ([feedbackMotor]) closes the loop and every
 * motor gets the same power. The controller runs here, not in the hub, so
 * its gains are plain [ShooterConfig] fields editable in Panels:
 *
 * `power = kF · target · (nominalVolts / battery) + kP · error + kI · ∫error + kD · d(error)/dt`
 *
 * clamped to 0..1: a flywheel is never braked by reverse power. Commands set
 * [targetRpm]; [periodic] reads the encoder and battery; [writeHardware]
 * computes and writes the power.
 *
 * Shooting speed eventually comes from [ShotModel.rpmForDistance]; until the
 * model is fitted, callers pass an RPM directly ([spinUp]).
 */
class ShooterSubsystem(
    private val motorNames: List<String> = RobotConfig.Shooter.MOTORS,
    private val feedbackMotor: String = RobotConfig.Shooter.FEEDBACK_MOTOR,
    private val clock: Clock = Clock.SYSTEM,
) : SubsystemBase("Shooter") {

    private var motors: List<DcMotorEx> = emptyList()
    private var encoder: DcMotorEx? = null
    private var voltageSensor: VoltageSensor? = null

    /** Commanded flywheel speed; null means stopped. Set by commands only. */
    var targetRpm: Double? = null
        private set

    /** Encoder-measured speed of [feedbackMotor]; 0.0 before init. */
    var measuredRpm: Double = 0.0
        private set

    /** Power written to every motor at the last [writeHardware]. */
    var power: Double = 0.0
        private set

    private var batteryVolts = Double.NaN
    private var lastVoltageReadNs = Long.MIN_VALUE
    private var integral = 0.0
    private var lastError = Double.NaN
    private var lastControlNs = Long.MIN_VALUE

    /** Close enough to [targetRpm] to take a shot. */
    val atTarget: Boolean
        get() {
            val target = targetRpm ?: return false
            return abs(measuredRpm - target) <= ShooterConfig.speedToleranceRpm
        }

    override fun init(hardwareMap: HardwareMap) {
        require(feedbackMotor in motorNames) { "Feedback motor \"$feedbackMotor\" is not one of $motorNames" }
        motors = motorNames.map { name ->
            hardwareMap.get(DcMotorEx::class.java, name).apply {
                direction = if (name in REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
                // FLOAT: braking a spinning flywheel loads the gearbox for nothing.
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
                // The encoder is still read; the velocity loop runs in writeHardware().
                mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
        encoder = motors[motorNames.indexOf(feedbackMotor)]
        voltageSensor = hardwareMap.voltageSensor.iterator().let { if (it.hasNext()) it.next() else null }
    }

    override fun periodic() {
        // Direction applies to the reading too: positive is launching speed.
        measuredRpm = (encoder?.velocity ?: 0.0) / ShooterConfig.ticksPerRev * 60.0
        // Battery voltage is not in the bulk read: one extra hub read every 100 ms.
        val now = clock.nanos()
        val sensor = voltageSensor
        if (sensor != null && (lastVoltageReadNs == Long.MIN_VALUE || now - lastVoltageReadNs >= 100_000_000L)) {
            batteryVolts = sensor.voltage
            lastVoltageReadNs = now
        }
    }

    override fun writeHardware() {
        val target = targetRpm
        power = if (target == null) {
            resetController()
            0.0
        } else {
            control(target)
        }
        motors.forEach { it.power = power }
    }

    private fun control(target: Double): Double {
        val c = ShooterConfig
        val now = clock.nanos()
        val error = target - measuredRpm
        val dt = if (lastControlNs == Long.MIN_VALUE) 0.0 else (now - lastControlNs) / 1e9
        val voltageScale = if (batteryVolts.isFinite() && batteryVolts > 6.0) c.nominalVolts / batteryVolts else 1.0
        if (dt > 0.0) {
            integral += error * dt
            // Anti-windup: the I term alone never exceeds iLimit power.
            if (c.kI > 0.0) integral = integral.coerceIn(-c.iLimit / c.kI, c.iLimit / c.kI)
        }
        val derivative = if (dt > 0.0 && lastError.isFinite()) (error - lastError) / dt else 0.0
        lastError = error
        lastControlNs = now
        val output = c.kF * target * voltageScale + c.kP * error + c.kI * integral + c.kD * derivative
        return output.coerceIn(0.0, 1.0)
    }

    private fun resetController() {
        integral = 0.0
        lastError = Double.NaN
        lastControlNs = Long.MIN_VALUE
    }

    /** Holds [ShooterConfig.targetRpm] until interrupted. */
    fun spinUp(): Command = spinUp { ShooterConfig.targetRpm }

    /** Holds [rpm], re-read every tick so a preset change applies while spinning, until interrupted. */
    fun spinUp(rpm: () -> Double): Command = logged(
        "Shooter spin up",
        infinite { targetRpm = rpm() }.requiring(this).setEnd { targetRpm = null },
    )

    override fun onCommandFault() {
        targetRpm = null
    }

    override fun stop() {
        targetRpm = null
        power = 0.0
        motors.forEach { it.power = 0.0 }
    }

    override fun health(): String =
        "${measuredRpm.toInt()}/${(targetRpm ?: 0.0).toInt()} RPM"

    override fun logState(log: StateLog) {
        log.put("targetRpm", targetRpm ?: Double.NaN)
        log.put("measuredRpm", measuredRpm)
        log.put("atTarget", atTarget)
        log.put("power", power)
        log.put("batteryVolts", batteryVolts)
    }

    private companion object {
        /**
         * Motor names whose positive power spins the flywheel backwards.
         * SDK convention: positive power on a FORWARD motor turns the shaft
         * counter-clockwise seen from the shaft end. "up" launches
         * clockwise (REVERSE), "down" counter-clockwise (FORWARD).
         */
        val REVERSED = setOf(RobotConfig.Shooter.UP_MOTOR)
    }
}

/** Flywheel settings, live-editable in Panels. Copy tuned values back here: nothing is persisted. */
@Configurable
object ShooterConfig {
    /** goBILDA 5203 1620 RPM: 28 ticks/rev × 3.7:1 gearbox, at the output shaft (goBILDA spec: 103.8). */
    @JvmField var ticksPerRev = 103.8

    /** Spin-up target for [ShooterSubsystem.spinUp] and the test op-modes. */
    @JvmField var targetRpm = 1400.0

    /** Match presets (motor output RPM), chosen on the operator's dpad. Placeholders until shots are tested. */
    @JvmField var nearRpm = 720.0
    @JvmField var farRpm = 1500.0

    /** |measured − target| at or under this counts as [ShooterSubsystem.atTarget]. */
    @JvmField var speedToleranceRpm = 60.0

    /**
     * Feedforward, power per RPM at [nominalVolts]. Start: 1 / 1620 (free
     * speed at 12 V); raise it until the flywheel settles near target with
     * kP = kI = 0, then add kP.
     */
    @JvmField var kF = 0.00062

    /** Power per RPM of error. */
    @JvmField var kP = 0.02
    @JvmField var kI = 0.0
    @JvmField var kD = 0.0

    /** Largest power the I term may contribute. */
    @JvmField var iLimit = 0.2

    /** Battery voltage [kF] was tuned at; kF is scaled by nominal / measured. */
    @JvmField var nominalVolts = 12.0
}

/**
 * Flywheel speed for a shot at the HIVE goal. The input is
 * `GoalObservation.horizontalDistanceIn`: horizontal distance from the turret
 * axis to the raised CELL's opening centre, inches.
 *
 * Not yet calibrated: fit this from shots at measured distances. Until then it
 * returns null, and callers must not spin the flywheel from it.
 */
object ShotModel {
    fun rpmForDistance(distanceIn: Double): Double? = null
}
