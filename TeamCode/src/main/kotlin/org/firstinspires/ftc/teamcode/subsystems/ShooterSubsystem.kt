package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.math.abs
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.StateLog
import org.firstinspires.ftc.teamcode.core.logging.logged

/**
 * Flywheel motors ([RobotConfig.Shooter.MOTORS] by default), all driven at one
 * target RPM. Velocity is closed-looped by the motor's built-in PID
 * (`RUN_USING_ENCODER`): commands set [targetRpm], [writeHardware] flushes it
 * as ticks/s, [periodic] reads the measured RPM back from the encoders.
 *
 * Shooting speed eventually comes from [ShotModel.rpmForDistance]; until the
 * model is fitted, callers pass an RPM directly ([spinUp]) and the test
 * op-mode tunes it live from Panels / the dpad.
 */
class ShooterSubsystem(
    private val motorNames: List<String> = RobotConfig.Shooter.MOTORS,
) : SubsystemBase("Shooter") {

    private var motors: List<DcMotorEx> = emptyList()

    /** PIDF last written to the motors; read from the hub at init. */
    private var appliedP = Double.NaN
    private var appliedI = Double.NaN
    private var appliedD = Double.NaN
    private var appliedF = Double.NaN

    /** Commanded flywheel speed; null means stopped. Set by commands only. */
    var targetRpm: Double? = null
        private set

    /** Encoder-measured speed, averaged over the motors; 0.0 before init. */
    var measuredRpm: Double = 0.0
        private set

    /** Close enough to [targetRpm] to take a shot. */
    val atTarget: Boolean
        get() {
            val target = targetRpm ?: return false
            return abs(measuredRpm - target) <= ShooterConfig.speedToleranceRpm
        }

    override fun init(hardwareMap: HardwareMap) {
        motors = motorNames.map { name ->
            hardwareMap.get(DcMotorEx::class.java, name).apply {
                direction = if (name in REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                mode = DcMotor.RunMode.RUN_USING_ENCODER
            }
        }
        // Baseline for applyPidfIfChanged: the hub firmware defaults of the
        // first motor. The SDK keeps no numeric defaults of its own.
        val pidf = motors.first().getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER)
        appliedP = pidf.p
        appliedI = pidf.i
        appliedD = pidf.d
        appliedF = pidf.f
    }

    override fun periodic() {
        measuredRpm = if (motors.isEmpty()) {
            0.0
        } else {
            motors.map { it.velocity / ShooterConfig.ticksPerRev * 60.0 }.average()
        }
    }

    override fun writeHardware() {
        applyPidfIfChanged()
        val target = targetRpm
        if (target == null) {
            motors.forEach { it.power = 0.0 }
        } else {
            val ticksPerSec = target / 60.0 * ShooterConfig.ticksPerRev
            motors.forEach { it.velocity = ticksPerSec }
        }
    }

    /**
     * Pushes [ShooterConfig]'s velocity PIDF to the motors when a Panels edit
     * changes it (a Lynx write, so only on change). A NaN slot keeps the
     * currently applied value — usually the hub default read at init. After
     * applying, the config fields are refreshed with the values actually in
     * effect so Panels edits start from what's live.
     */
    private fun applyPidfIfChanged() {
        val c = ShooterConfig
        if (c.velocityP == appliedP && c.velocityI == appliedI && c.velocityD == appliedD && c.velocityF == appliedF) {
            return
        }
        val p = if (c.velocityP.isNaN()) appliedP else c.velocityP
        val i = if (c.velocityI.isNaN()) appliedI else c.velocityI
        val d = if (c.velocityD.isNaN()) appliedD else c.velocityD
        val f = if (c.velocityF.isNaN()) appliedF else c.velocityF
        motors.forEach { it.setVelocityPIDFCoefficients(p, i, d, f) }
        appliedP = p
        appliedI = i
        appliedD = d
        appliedF = f
        c.velocityP = p
        c.velocityI = i
        c.velocityD = d
        c.velocityF = f
    }

    /** Holds [ShooterConfig.targetRpm] until interrupted. */
    fun spinUp(): Command = logged(
        "Shooter spin up",
        infinite { targetRpm = ShooterConfig.targetRpm }.requiring(this).setEnd { targetRpm = null },
    )

    override fun onCommandFault() {
        targetRpm = null
    }

    override fun stop() {
        targetRpm = null
        motors.forEach { it.power = 0.0 }
    }

    override fun health(): String =
        "${measuredRpm.toInt()}/${(targetRpm ?: 0.0).toInt()} RPM"

    override fun logState(log: StateLog) {
        log.put("targetRpm", targetRpm ?: Double.NaN)
        log.put("measuredRpm", measuredRpm)
        log.put("atTarget", atTarget)
        log.put("pidfP", appliedP)
        log.put("pidfI", appliedI)
        log.put("pidfD", appliedD)
        log.put("pidfF", appliedF)
    }

    private companion object {
        /**
         * Motor names whose positive velocity spins the flywheel backwards.
         * The two flywheel motors are mounted mirrored, so the right motor is
         * reversed; both then drive and report speed in flywheel direction.
         */
        val REVERSED = setOf(RobotConfig.Shooter.RIGHT_MOTOR)
    }
}

/** Flywheel settings, live-editable in Panels. */
@Configurable
object ShooterConfig {
    /** goBILDA 5203 1620 RPM: 28 ticks/rev × 3.7:1 gearbox, at the output shaft. */
    @JvmField var ticksPerRev = 103.6

    /** Spin-up target for [ShooterSubsystem.spinUp] and the test op-mode; below the motor's 1620 RPM max. */
    @JvmField var targetRpm = 1400.0

    /** |measured − target| at or under this counts as [ShooterSubsystem.atTarget]. */
    @JvmField var speedToleranceRpm = 60.0

    /**
     * Velocity PIDF for the RUN_USING_ENCODER loop. NaN means "keep the
     * currently applied value" — at init that is the hub firmware default
     * (the SDK keeps no numeric defaults of its own). Any edit applies to
     * all flywheel motors immediately, and the fields refresh to what's in
     * effect, so tune from Panels, watch `Shooter/measuredRpm`, then type
     * the working numbers here: the hub forgets them on a power cycle.
     */
    @JvmField var velocityP = Double.NaN
    @JvmField var velocityI = Double.NaN
    @JvmField var velocityD = Double.NaN
    @JvmField var velocityF = Double.NaN
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
