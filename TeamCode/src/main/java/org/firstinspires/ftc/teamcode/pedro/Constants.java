package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.DriveConfig;

/**
 * Pedro Pathing 3 constants for this robot, in the layout AutoTune generates
 * (https://pedropathing.com/docs/pathing/tuning/constants). Paste AutoTune's
 * Java output over the matching block; keep the hardware names pointing at
 * {@link RobotConfig}.
 *
 * <p>Provenance of every value:
 * <ul>
 *   <li>Motor names and directions: ported from the team's Quickstart project
 *       (verified on hardware 2026-09-14). Re-check with AutoTune's Mecanum
 *       procedure on the competition chassis.</li>
 *   <li>Pinpoint offsets and directions: the team's 2026-11-01 Pinpoint Tuner
 *       run (the tuner with the read-before-Stop direction fix, now upstream in
 *       Pedro-Pathing/Quickstart) and a Drive Only push test. The tuner can flip
 *       an offset's sign (Pedro issue #185): an in-place spin should hold
 *       position within about 1 in. Pedro 2's forwardPodY/strafePodX map to
 *       xPodOffset/yPodOffset: both versions pass them to
 *       {@code GoBildaPinpointDriver.setOffsets(x, y)} in that order.</li>
 *   <li>Foresight: AutoTune's Foresight procedure output, pasted verbatim into
 *       {@link #foresightConfig}; {@link #FORESIGHT_TUNED} is true. These values
 *       predate voltage compensation: re-run through compensation at 12.0 V after
 *       the 2026-10-25 sync, and after any drivetrain or weight change.</li>
 * </ul>
 *
 * <p>Pedro 3 has no voltage compensation for mecanum; {@link VoltageCompensatedMecanum}
 * adds it, for the op-modes and AutoTune alike.
 */
public final class Constants {

    private Constants() {}

    /**
     * True once {@link #foresightConfig} holds AutoTune output. If it is ever
     * flipped back to false the follower has no path algorithm: manual driving
     * works, and path, hold and turn commands refuse to start.
     */
    public static final boolean FORESIGHT_TUNED = true;

    /**
     * The battery voltage {@link #foresightConfig} is expressed at: every follower power
     * is scaled by NOMINAL_VOLTS / battery volts. Keep this at 12.0 while re-running
     * AutoTune through the same compensation; do not copy another robot's loaded voltage.
     */
    public static final double NOMINAL_VOLTS = 12.0;

    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConfig.Drive.FRONT_LEFT_MOTOR);
        c.frontRightName.set(RobotConfig.Drive.FRONT_RIGHT_MOTOR);
        c.backLeftName.set(RobotConfig.Drive.BACK_LEFT_MOTOR);
        c.backRightName.set(RobotConfig.Drive.BACK_RIGHT_MOTOR);

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        // Latest measured offsets. Both directions confirmed REVERSED by the user's
        // 2026-11-01 re-run of the Pinpoint Tuner and the Drive Only push test.
        c.name.set(RobotConfig.Localization.PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.733931744192529);
        c.yPodOffset.set(3.517399585153174);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
    });


    /** AutoTune's Foresight procedure output, pasted verbatim. */
    public static final ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3149776271663011);
                Controller secondaryTranslationalForward = Controller.proportional(0.11637581688074408);
                Controller primaryTranslationalLateral = Controller.proportional(0.4802387111841137);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17743537156783293);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014597226061475232));
                c.brake.set(Controller.proportionalFeedforward(0.012407642152253947));

                c.headingFeedback.set(Controller.proportional(5.047861924311455));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05866924639208644, 0.003280217577239187));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11223205639513174, 0.06518828391149664));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.198632343122721E-4, 0.0015564455133477472));

                c.maxAchievableForwardVelocity.set(80.77620101715345);
                c.maxAchievableStrafeVelocity.set(63.93016612200878);
                c.naturalForwardDeceleration.set(33.11195066325026);
                c.naturalStrafeDeceleration.set(66.0621890115333);
            }
    );

    /**
     * Builds the follower: Pinpoint localizer, mecanum drivetrain, and Foresight
     * once tuned. Call once per op-mode.
     *
     * <p>The Pinpoint is read synchronously inside {@code Follower.update()}.
     * Moving that I2C read to a background thread was tried and reverted: the
     * Pinpoint shares the Control Hub's Lynx serial link with the drive-motor
     * writes, so a background poll starves on link contention (~12–25 ms per
     * read) while the main loop spins on a stale pose.
     */
    public static Follower create(HardwareMap hardwareMap, DoubleSupplier batteryVolts) {
        drivetrainConfig.manualBrakeMode.set(DriveConfig.brakeOnTeleop);
        return new Follower(
                createLocalizer(hardwareMap, localizerConfig),
                drivetrain(hardwareMap, batteryVolts),
                FORESIGHT_TUNED ? new Foresight(foresightConfig) : null
        );
    }

    public static VoltageCompensatedMecanum drivetrain(HardwareMap hardwareMap, DoubleSupplier batteryVolts) {
        return new VoltageCompensatedMecanum(hardwareMap, drivetrainConfig, NOMINAL_VOLTS, batteryVolts);
    }

    /** Shared by runtime and AutoTune; a pasted config must never re-enable IMU recalibration. */
    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap, PinpointConfig config) {
        config.resetMode.set(PinpointLocalizer.ResetMode.NONE);
        return new PinpointLocalizer(hardwareMap, config);
    }

}
