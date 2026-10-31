package org.firstinspires.ftc.teamcode.subsystems

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.infinite
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
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
            val l = left()
            val r = right()
            require(l.isFinite() && r.isFinite()) { "Shovel positions must be finite" }
            leftPosition = l.coerceIn(0.0, 1.0)
            rightPosition = r.coerceIn(0.0, 1.0)
            enabled = true
        }.requiring(this).setEnd { enabled = false },
    )

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
