package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(7)
            .forwardZeroPowerAcceleration(-46.913167084947325)
            .lateralZeroPowerAcceleration(-84.9611956100311)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(.1,
                    0,
                    0.01,
                    0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.075,
                    0,
                    0.05,
                    0))
            .headingPIDFCoefficients(new PIDFCoefficients( 2,
                    0.001,
                    0.1,
                    0.001))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,
                    0.001,
                    0.1,
                    0.001))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03,
                    0,
                    0.0001,
                    0.6,
                    0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,
                    0,
                    0.0005,
                    0.6,
                    0))
            .centripetalScaling(0.005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(66.77182271161418)
            .yVelocity(46.7488557109);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.5)
            .strafePodX(-8)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
