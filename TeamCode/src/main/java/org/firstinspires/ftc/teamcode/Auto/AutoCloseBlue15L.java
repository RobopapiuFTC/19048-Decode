package org.firstinspires.ftc.teamcode.Auto;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Disabled
@Autonomous(name="Auto Close Blue 15 NEW", group="Blue")
public class AutoCloseBlue15L extends OpMode{
    public HubBulkRead bulk;
    private TelemetryManager t;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Robot r;
    private boolean okp,okf;

    private int pathState;
    private  Pose startPose = new Pose(20, 125, Math.toRadians(234));
    private  Pose scorePose = new Pose(57, 79, Math.toRadians(210));
    private  Pose doorPose = new Pose(14.5,65,Math.toRadians(153));
    private  Pose doorM = new Pose(14,53,Math.toRadians(153));
    private  Pose line1Pose = new Pose(13, 84, Math.toRadians(180));
    private  Pose line2Pose = new Pose(15, 60, Math.toRadians(180));
    private  Pose line3Pose = new Pose(8, 36, Math.toRadians(180));
    public  Pose endPose = new Pose(60,108,Math.toRadians(180));
    public double aimPose = 0;
    private PathChain scorePreload,doorPickup,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end,scoreDoor,doorMove;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(57,56),
                                new Pose(46,60),
                                line2Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(), line2Pose.getHeading())
                .build();
        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line2Pose.getHeading(), scorePose.getHeading(),0.2)
                .build();
      /*  doorPickup = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(44,62),
                                doorPose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), doorPose.getHeading(),0.3)
                .build();
        doorMove = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, doorM)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), doorM.getHeading())
                .build();
        scoreDoor = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading())
                .build(); */
        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(51,89),
                                new Pose(37,83),
                                line1Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(), line1Pose.getHeading(),0.2)
                .build();
        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line1Pose.getHeading(), Math.toRadians(270))
                .build();
        grabPickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(58,29),
                                new Pose(44,36),
                                line3Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(Math.toRadians(270), line3Pose.getHeading(),0.4,0.1)
                .build();
        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, endPose)
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation().setReversed()
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                r.aiming=false;
                r.aim=true;
                r.pids=true;
                okp=true;
                okf=true;
                r.shoot=true;
                r.oks=true;
                nextPath();
                break;
            case 1:
                if(follower.getPose().getY()<105 && okf){
                    r.tu.setYaw(r.tu.Yaw(r.getShootTarget(),scorePose));
                    okf=false;
                }
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
                        okf=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    r.s.on();
                    okf=true;
                    r.aim=true;
                    r.tu.setYaw(r.tu.Yaw(r.getShootTarget(),scorePose));
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
                                    new BezierCurve(follower::getPose,
                                            new Pose(44,62),
                                            doorPose)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), doorPose.getHeading(),0.3)
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
                    doorMove = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower::getPose, doorM)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), doorM.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>0.1) {
                        follower.followPath(doorMove, true);
                        okp=true;
                        nextPath();
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(follower::getPose, scorePose)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePose.getHeading(),0.2)
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
                        r.tu.setYaw(r.tu.Yaw(r.getShootTarget(),scorePose));
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
            case 7:
                if(!follower.isBusy()) {
                    r.s.on();
                    okf=true;
                    okp=true;
                    r.aim=true;
                    r.tu.setYaw(r.tu.Yaw(r.getShootTarget(),new Pose(scorePose.getX(),scorePose.getY(),Math.toRadians(270))));
                    follower.followPath(scorePickup1,true);
                    nextPath();
                }
                break;
            case 8:
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
                        follower.followPath(grabPickup3,true);
                        nextPath();
                    }
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    r.s.on();
                    okf=true;
                    okp=true;
                    follower.followPath(end,true);
                    nextPath();
                }
                break;
            case 10:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(follower.getPose().getX()>40){
                    r.aim=true;
                    r.aiming=true;
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
            case 11:
                if(!follower.isBusy()) {
                    r.i.pornit=false;
                    r.aim=false;
                    r.aima=false;
                    r.tu.setTurretTarget(0);
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
        bulk.clearCache(HubBulkRead.Hubs.ALL);
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
        bulk = new HubBulkRead(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,true,true,startPose);
        r.aInit();
        r.setShootTarget();
        r.s.shootc=970;
        //  r.s.offset=-20;
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
