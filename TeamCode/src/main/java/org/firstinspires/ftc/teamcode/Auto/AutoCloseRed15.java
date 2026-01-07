package org.firstinspires.ftc.teamcode.Auto;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto Close Red 15", group="Red")
public class AutoCloseRed15 extends OpMode{
    private TelemetryManager t;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Robot r;
    private boolean okp;

    private int pathState;
    private final Pose goalPose = new Pose(-4,144,0);
    private final Pose startPose = new Pose(144-20, 125, Math.toRadians(54));
    private final Pose scorePose = new Pose(144-54, 96, Math.toRadians(0));
    private final Pose doorPose = new Pose(144-14,63.7,Math.toRadians(32));
    private final Pose line1Pose = new Pose(144-17, 84, Math.toRadians(0));
    private final Pose line2Pose = new Pose(144-15, 62, Math.toRadians(0));
    private final Pose line3Pose = new Pose(144-17, 36, Math.toRadians(0));
    public final Pose endPose = new Pose(144-36,90,Math.toRadians(0));
    private PathChain scorePreload,doorPickup,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end,scoreDoor;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(scorePose,
                                new Pose(144-55,86),
                                line1Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line1Pose.getHeading())
                .build();
        doorPickup = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(scorePose,
                                new Pose(144-55, 60),
                                doorPose)
                )
                .setNoDeceleration()
                .setLinearHeadingInterpolation(scorePose.getHeading(),doorPose.getHeading())
                .build();
        scoreDoor = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                doorPose,
                                new Pose(144-55, 60),
                                scorePose
                        )
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(doorPose.getHeading(),scorePose.getHeading())
                .build();
        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(line1Pose,scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line1Pose.getHeading(),scorePose.getHeading())
                .build();

        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(144-59, 60),
                                line2Pose
                        )
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line2Pose.getHeading())
                .build();

        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                line2Pose,
                                new Pose(144-59, 53),
                                scorePose
                        )
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line2Pose.getHeading(),scorePose.getHeading())
                .build();

        grabPickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(144-83.483, 39),
                                line3Pose
                        )
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line3Pose.getHeading())
                .build();


        scorePickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(line3Pose,scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line3Pose.getHeading(),scorePose.getHeading())
                .build();

        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(scorePose,endPose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(),endPose.getHeading())
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                r.pids=true;
                okp=true;
                r.shooter();
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.4) {
                        follower.followPath(grabPickup2, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2,true);
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
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.4) {
                        follower.followPath(doorPickup, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.2) {
                        follower.followPath(scoreDoor, true);
                        r.intake=false;
                        r.oki=false;
                        r.i.pornit=false;
                        okp=true;
                        setPathState(5);
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.4) {
                        follower.followPath(grabPickup1, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.4) {
                        follower.followPath(doorPickup, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        setPathState(8);
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.2) {
                        follower.followPath(scoreDoor, true);
                        r.intake=false;
                        r.oki=false;
                        r.i.pornit=false;
                        okp=true;
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()<0.1){
                        r.shooter();
                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.i.pornit=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.4) {
                        follower.followPath(end, true);
                        okp=true;
                        r.aiming=false;
                        setPathState(10);
                    }
                }
                break;
            case 10:
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
        telemetry.addData("Target Velocity", r.s.getTarget());
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
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,false,true,startPose);
        r.aInit();
        r.setShootTarget();
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
