package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
 *   <li>Pinpoint offsets and directions: AutoTune's Pinpoint Tuner output, run
 *       on this robot. Pedro 2's forwardPodY/strafePodX map to
 *       xPodOffset/yPodOffset: both versions pass them to
 *       {@code GoBildaPinpointDriver.setOffsets(x, y)} in that order.</li>
 *   <li>Foresight: AutoTune's Foresight procedure output, pasted verbatim into
 *       {@link #foresightConfig}; {@link #FORESIGHT_TUNED} is true. Re-run the
 *       procedure and re-paste after any drivetrain or weight change.</li>
 * </ul>
 *
 * <p>Pedro 2's voltage compensation has no Pedro 3 equivalent.
 */
public final class Constants {

    private Constants() {}

    /**
     * True once {@link #foresightConfig} holds AutoTune output. If it is ever
     * flipped back to false the follower has no path algorithm: manual driving
     * works, and path, hold and turn commands refuse to start.
     */
    public static final boolean FORESIGHT_TUNED = true;

    public static final MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotConfig.Drive.FRONT_LEFT_MOTOR);
        c.backLeftName.set(RobotConfig.Drive.BACK_LEFT_MOTOR);
        c.frontRightName.set(RobotConfig.Drive.FRONT_RIGHT_MOTOR);
        c.backRightName.set(RobotConfig.Drive.BACK_RIGHT_MOTOR);

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static final PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(RobotConfig.Localization.PINPOINT);
        // Pinpoint Tuner output, run on this robot. Re-run and re-paste after
        // moving the pod assembly.
        c.xPodOffset.set(1.4569476270300197);
        c.yPodOffset.set(-0.9436713241216704);
        c.offsetUnits.set(DistanceUnit.INCH);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        // X REVERSED: the pod assembly faces backward. The Pinpoint Tuner's
        // FORWARD suggestion read the X pod with the wrong sign on this robot;
        // confirmed by pushing the robot and watching the Panels field view.
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // No device command at construction. LocalizerSubsystem writes the start
        // pose at every INIT instead. RECALIBRATE_IMU (Pedro's default) and
        // RESET_AND_RECALIBRATE_IMU would recalibrate the gyro every run, which
        // needs a stationary robot for ~0.25 s and is not needed to zero the pose.
        c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
    });

    /** AutoTune's Foresight procedure output, pasted verbatim. */
    public static final ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.15325564047848608);
        Controller secondaryTranslationalForward = Controller.proportional(0.05662386409066711);
        Controller primaryTranslationalLateral = Controller.proportional(0.24534267817422847);
        Controller secondaryTranslationalLateral = Controller.proportional(0.09064756390827897);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.018564902717508547));
        c.brake.set(Controller.proportionalFeedforward(0.015780167309882263));

        c.headingFeedback.set(Controller.proportional(1.5988025611530874));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.02260860499841661, 0.010232454306551413));

        c.linearBrakeCoefficients.set(Matrix.diag(0.03690990258732756, -0.006698982990924054));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.002431382843941635, 0.0034439014976842437));

        c.maxAchievableForwardVelocity.set(57.37232242800095);
        c.maxAchievableStrafeVelocity.set(49.99157880573599);
        c.naturalForwardDeceleration.set(61.98385527089174);
        c.naturalStrafeDeceleration.set(83.19735660411557);
    });

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
    public static Follower create(HardwareMap hardwareMap) {
        drivetrainConfig.manualBrakeMode.set(DriveConfig.brakeOnTeleop);
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                FORESIGHT_TUNED ? new Foresight(foresightConfig) : null
        );
    }
}
