package org.firstinspires.ftc.teamcode.Auto;
import com.bylazar.telemetry.TelemetryManager;
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

import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto Far Blue", group="Blue")
public class AutoFarBlue extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean okp;
    private Robot r;
    private TelemetryManager t;

    private int pathState;
    private final Pose goalPose = new Pose(0,144,0);
    private final Pose startPose = new Pose(56, 9, Math.toRadians(90));
    private final Pose scorePose = new Pose(55, 20, Math.toRadians(90));
    private final Pose positionPose= new Pose(36,17,Math.toRadians(210));
    private final Pose line1Pose = new Pose(11, 6, Math.toRadians(210));
    private final Pose line2Pose = new Pose(7, 18, Math.toRadians(210));
    public final Pose endPose = new Pose(60,35,Math.toRadians(0));
    private PathChain scorePreload,position,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                       new BezierLine(startPose, scorePose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        position = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose,positionPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(200))
                .build();
        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(positionPose,
                                new Pose(7.000, 34),
                                line1Pose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(200))
                .build();

        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(line1Pose,scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(200),Math.toRadians(180))
                .build();

        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(8, -10.000),
                                line2Pose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(210))
                .build();

        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                line2Pose,
                                new Pose(39, 12),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(210),Math.toRadians(180))
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
                okp=true;
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        follower.followPath(position, true);
                        r.intake();
                        okp=true;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup1,true);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        follower.followPath(grabPickup2, true);
                        okp=true;
                        r.intake();
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        okp=true;
                        r.intake();
                        follower.followPath(grabPickup2,true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        okp=true;
                        r.i.pornit=false;
                        r.aim=false;
                        follower.followPath(end, true);
                        setPathState(9);
                    }

                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    r.aim=false;
                    r.aima=false;
                    r.tu.set(0);
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
        r.aPeriodic();
        autonomousPathUpdate();

        t.addData("path state", pathState);
        t.addData("Follower Pose", r.f.getPose().toString());
        t.addData("Velocity: ", r.s.getVelocity());
        t.update(telemetry);
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,true,true);
        r.aInit();

    }
    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop() {
        r.stop();
    }
}
