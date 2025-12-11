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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto Close Blue", group="Blue")
public class AutoCloseBlue extends OpMode{
    private TelemetryManager t;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Robot r;
    private boolean okp;

    private int pathState;
    private final Pose goalPose = new Pose(-4,144,0);
    private final Pose startPose = new Pose(20, 130, Math.toRadians(234));
    private final Pose scorePose = new Pose(54, 91, Math.toRadians(180));
    private final Pose doorPose = new Pose(18,81,Math.toRadians(90));
    private final Pose line1Pose = new Pose(16, 91, Math.toRadians(180));
    private final Pose line2Pose = new Pose(11, 65, Math.toRadians(0));
    private final Pose line3Pose = new Pose(11, 43, Math.toRadians(0));
    public final Pose endPose = new Pose(36,75,Math.toRadians(0));
    private PathChain scorePreload,doorPickup,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, scorePose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(234), Math.toRadians(180))
                .build();

        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose,line1Pose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(15)
                .build();
        doorPickup = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(line1Pose,
                                new Pose(38,84),
                                        doorPose)
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(doorPose,scorePose)
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
                .setVelocityConstraint(10)
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
                                new Pose(83.483, 35),
                                line3Pose
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setVelocityConstraint(10)
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
                        follower.followPath(grabPickup1, true);
                        r.intake();
                        okp=true;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                            follower.followPath(doorPickup,true);
                            setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(scorePickup1, true);
                        okp=true;
                        setPathState(4);
                    }
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
                   // r.i.pornit=false;
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
                        follower.followPath(grabPickup3,true);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                   // r.i.pornit=false;
                    follower.followPath(scorePickup3, true);
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
                    r.tu.setYaw(0);
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
        telemetry.addData("Follower Pose: ",follower.getPose().toString());
        telemetry.addData("Dist: ", r.dist);
        telemetry.addData("Velocity: ",r.s.getVelocity());
        telemetry.addData("Turret Ticks: ", r.tu.getTurret());
        telemetry.addData("Turret Target: ",r.tu.getTurretTarget());
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
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,true,true,startPose);
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
