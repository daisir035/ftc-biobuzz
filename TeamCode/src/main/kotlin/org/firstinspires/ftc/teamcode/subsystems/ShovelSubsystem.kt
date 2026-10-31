package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.max
import kotlin.math.min
import org.firstinspires.ftc.teamcode.RobotConfig
import org.firstinspires.ftc.teamcode.core.SubsystemBase
import org.firstinspires.ftc.teamcode.core.logging.logged

/** Mirrored positional goBILDA 25-2 shovel servos, using their 500–2500 us range. */
class ShovelSubsystem : SubsystemBase("Shovel") {
    private var servos: List<ServoImplEx> = emptyList()

    var enabled = false
        private set
    var leftPosition = Double.NaN
        private set
    var rightPosition = Double.NaN
        private set
    /** Applied servo rotation relative to the aligned reference, not the raised position. */
    var offsetDegrees = Double.NaN
        private set

    override fun init(hardwareMap: HardwareMap) {
        servos = listOf(RobotConfig.Shovel.LEFT_SERVO, RobotConfig.Shovel.RIGHT_SERVO)
            .map { hardwareMap.get(ServoImplEx::class.java, it) }
        servos.forEachIndexed { index, servo ->
            servo.setPwmDisable()
            servo.direction = if (index == 0) Servo.Direction.REVERSE else Servo.Direction.FORWARD
            servo.pwmRange = PwmRange(500.0, 2500.0)
        }
    }

    /** Targets are logical positions (left REVERSE, right FORWARD), not measured angles. */
    fun holdPositions(left: () -> Double, right: () -> Double): Command = logged(
        "Shovel calibration hold",
        infinite {
            offsetDegrees = Double.NaN
            stagePositions(left(), right())
        }.requiring(this).setEnd { enabled = false },
    )

    /** Both logical positions advance together; either endpoint limits the shared rotation. */
    fun holdOffsetDegrees(degrees: () -> Double): Command = logged(
        "Shovel angle hold",
        infinite {
            val left = ShovelConfig.leftAlignedPosition
            val right = ShovelConfig.rightAlignedPosition
            val requested = degrees()
            val fullyLowered = ShovelConfig.fullyLoweredOffsetDegrees
            require(left in 0.0..1.0 && right in 0.0..1.0 && requested.isFinite() &&
                fullyLowered.isFinite() && fullyLowered >= 0.0
            ) {
                "Shovel alignment must be in 0..1 and angle must be finite"
            }
            offsetDegrees = requested.coerceIn(
                -min(left, right) * 300.0,
                min((1.0 - max(left, right)) * 300.0, fullyLowered),
            )
            stagePositions(left + offsetDegrees / 300.0, right + offsetDegrees / 300.0)
        }.requiring(this).setEnd { enabled = false },
    )

    private fun stagePositions(left: Double, right: Double) {
        require(left.isFinite() && right.isFinite()) { "Shovel positions must be finite" }
        leftPosition = left.coerceIn(0.0, 1.0)
        rightPosition = right.coerceIn(0.0, 1.0)
        enabled = true
    }

    override fun writeHardware() {
        servos.forEachIndexed { index, servo ->
            if (enabled) {
                servo.position = if (index == 0) leftPosition else rightPosition
                servo.setPwmEnable()
            } else {
                servo.setPwmDisable()
            }
        }
    }

    override fun onCommandFault() = stop()

    override fun stop() {
        enabled = false
        servos.forEach { it.setPwmDisable() }
    }
}

/** Measured alignment only: these positions have not been calibrated as fully raised. */
@Configurable
object ShovelConfig {
    @JvmField var leftAlignedPosition = 0.36
    @JvmField var rightAlignedPosition = 0.225
    @JvmField var targetOffsetDegrees = 0.0
    @JvmField var fullyLoweredOffsetDegrees = 133.0
    @JvmField var preset1OffsetDegrees = 0.0
    @JvmField var preset2OffsetDegrees = 125.0
    @JvmField var stepDegrees = 1.0
}
