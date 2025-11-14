package org.firstinspires.ftc.teamcode.Auto;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto Close Blue", group="Blue")
public class AutoCloseBlue extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose goalPose = new Pose(6,140,0);
    private final Pose startPose = new Pose(20, 130, Math.toRadians(234));
    private final Pose scorePose = new Pose(60, 84, Math.toRadians(180));
    private final Pose line1Pose = new Pose(18, 84, Math.toRadians(180));
    private final Pose line2Pose = new Pose(12, 60, Math.toRadians(0));
    private final Pose line3Pose = new Pose(12, 36, Math.toRadians(0));
    public final Pose endPose = new Pose(36,60,Math.toRadians(0));
    private PathChain scorePreload,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(180))
                .build();

        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose,line1Pose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(line1Pose,scorePose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(69.905, 59.176),
                                line2Pose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                line2Pose,
                                new Pose(66.719, 46.100),
                                scorePose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grabPickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(83.483, 30.678),
                                line3Pose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        scorePickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(line3Pose,scorePose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose,endPose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(end, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {}
}
