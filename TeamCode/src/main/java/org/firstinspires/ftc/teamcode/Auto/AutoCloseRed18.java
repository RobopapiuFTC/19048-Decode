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

@Autonomous(name="Auto Close Red 18", group="Red")
public class AutoCloseRed18 extends OpMode{
    private TelemetryManager t;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Robot r;
    private boolean okp,okf;

    private int pathState;
    private  Pose startPose = new Pose(20, 125, Math.toRadians(234));
    private  Pose scorePose = new Pose(54, 96, Math.toRadians(180));
    private  Pose doorPose = new Pose(15,65,Math.toRadians(180));
    private  Pose doorM = new Pose(14,53,Math.toRadians(153));
    private  Pose line1Pose = new Pose(13.5, 84, Math.toRadians(180));
    private  Pose line2Pose = new Pose(10, 59, Math.toRadians(180));
    private  Pose line3Pose = new Pose(10, 36, Math.toRadians(180));
    public  Pose endPose = new Pose(36,90,Math.toRadians(180));
    private PathChain scorePreload,doorPickup,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end,scoreDoor,doorMove;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(startPose.getHeading(),scorePose.getHeading())
                .build();

        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(55,86).mirror(),
                                line1Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line1Pose.getHeading())
                .build();
        doorMove = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,new Pose(18.000, 55.000).mirror(),
                                doorM)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(doorPose.getHeading(),doorM.getHeading())
                .build();
        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,
                                new Pose(60,108,Math.toRadians(180)).mirror())
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line1Pose.getHeading(),scorePose.getHeading())
                .build();

        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(57.667, 51.464).mirror(),
                                new Pose(54.802, 60.557).mirror(),
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
                                follower::getPose,
                                new Pose(59, 60).mirror(),
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
                                follower::getPose,
                                new Pose(83.483, 39).mirror(),
                                line3Pose
                        )
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line3Pose.getHeading())
                .build();


        scorePickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,
                                new Pose(60,108,Math.toRadians(180)).mirror())
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line3Pose.getHeading(),Math.toRadians(180))
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                r.pids=true;
                okp=true;
                r.shooter();
                nextPath();
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(grabPickup2, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    r.aim=true;
                    r.aiming=true;
                    r.s.on();
                    okf=true;
                    follower.followPath(scorePickup2,true);
                    nextPath();
                }
                break;

            case 3:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {

                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower.getPose(),
                                            new Pose(55, 60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(scorePose.getHeading(),doorPose.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(doorPickup, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(doorMove,true);
                    nextPath();
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            follower.getPose(),
                                            new Pose(55, 60),
                                            scorePose
                                    )
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(doorPose.getHeading(),scorePose.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.5) {
                        follower.followPath(scoreDoor, true);
                        r.intake=false;
                        r.oki=false;
                        r.aim=true;
                        r.aiming=true;
                        r.s.on();
                        okf=true;
                        okp=true;
                        nextPath();
                    }
                }
                break;
            case 6:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {

                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower.getPose(),
                                            new Pose(55, 60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(scorePose.getHeading(),doorPose.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(doorPickup, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(doorMove,true);
                    nextPath();
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            follower.getPose(),
                                            new Pose(55, 60),
                                            scorePose
                                    )
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(doorPose.getHeading(),scorePose.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.5) {
                        follower.followPath(scoreDoor, true);
                        r.intake=false;
                        r.oki=false;
                        r.aim=true;
                        r.aiming=true;
                        r.s.on();
                        okf=true;
                        okp=true;
                        nextPath();
                    }
                }
                break;
            case 9:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {

                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower.getPose(),
                                            new Pose(55, 60),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(scorePose.getHeading(),doorPose.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(doorPickup, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    follower.followPath(doorMove,true);
                    nextPath();
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(
                                            follower.getPose(),
                                            new Pose(55, 60),
                                            scorePose
                                    )
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(doorPose.getHeading(),scorePose.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.5) {
                        follower.followPath(scoreDoor, true);
                        r.intake=false;
                        r.oki=false;
                        r.aim=true;
                        r.aiming=true;
                        r.s.on();
                        okf=true;
                        okp=true;
                        nextPath();
                    }
                }
                break;
            case 12:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(grabPickup1,true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    r.aim=true;
                    r.aiming=true;
                    r.s.on();
                    okf=true;
                    follower.followPath(scorePickup1,true);
                    nextPath();
                }
                break;
            case 14:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    r.i.pornit=false;
                    r.aim=false;
                    r.aima=false;
                    r.tu.setYaw(0);
                    endPath();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void nextPath(){
        pathState++;
        pathTimer.resetTimer();
    }

    public void endPath(){
        pathState=-1;
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

        startPose = startPose.mirror();
        scorePose = scorePose.mirror();
        doorPose = doorPose.mirror();
        doorM = doorM.mirror();
        line1Pose = line1Pose.mirror();
        line2Pose = line2Pose.mirror();
        line3Pose = line3Pose.mirror();
        endPose = endPose.mirror();

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
