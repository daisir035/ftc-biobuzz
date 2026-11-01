package org.firstinspires.ftc.teamcode.pedro

import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.pedropathing.revhub.localizers.PinpointConfig
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple
import org.firstinspires.ftc.teamcode.core.sim.FakeHardwareMap
import org.firstinspires.ftc.teamcode.core.sim.deviceProxy
import java.nio.ByteBuffer
import java.nio.ByteOrder
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class PinpointInitializationTest {
    @Test fun pastedConfigKeepsTheImuCalibration() {
        assertEquals(PinpointLocalizer.ResetMode.NONE, Constants.localizerConfig.resetMode.get())
    }

    @Test fun sharedFactoryDoesNotWriteEitherImuResetCommandEvenAfterPastingDefaults() {
        val controlWrites = mutableListOf<Int>()
        val pinpoint = GoBildaPinpointDriver(deviceProxy(I2cDeviceSynchSimple::class.java) { name, args ->
            when (name) {
                "write" -> {
                    if (args[0] == 4) controlWrites += ByteBuffer.wrap(args[1] as ByteArray)
                        .order(ByteOrder.LITTLE_ENDIAN).int
                    null
                }
                "read" -> {
                    val bytes = ByteBuffer.allocate(args[1] as Int).order(ByteOrder.LITTLE_ENDIAN)
                    when (args[0]) {
                        2 -> bytes.putInt(2)
                        18 -> { bytes.putInt(1); bytes.putInt(1000) }
                    }
                    bytes.array()
                }
                else -> null
            }
        }, false)
        val hardware = FakeHardwareMap().apply { put("pinpoint", pinpoint) }
        val pasted = PinpointConfig {
            it.name.set("pinpoint")
            it.xPodOffset.set(0.0)
            it.yPodOffset.set(0.0)
            it.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            it.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        }
        assertEquals(PinpointLocalizer.ResetMode.RECALIBRATE_IMU, pasted.resetMode.get())
        Constants.createLocalizer(hardware, pasted)
        assertEquals(PinpointLocalizer.ResetMode.NONE, pasted.resetMode.get())
        // Real SDK DEVICE_CONTROL bits 0/1 trigger recalibration/reset; direction writes use other bits.
        assertTrue(controlWrites.isNotEmpty())
        assertTrue(controlWrites.none { it and 3 != 0 })
    }

}
