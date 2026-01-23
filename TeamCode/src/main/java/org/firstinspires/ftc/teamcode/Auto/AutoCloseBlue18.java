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

@Autonomous(name="Auto Close Blue 18", group="Blue")
public class AutoCloseBlue18 extends OpMode{
    private TelemetryManager t;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Robot r;
    private boolean okp,okf;

    private int pathState;
    private final Pose startPose = new Pose(20, 125, Math.toRadians(234));
    private final Pose scorePose = new Pose(54, 96, Math.toRadians(180));
    private final Pose doorPose = new Pose(13,62,Math.toRadians(153));
    private final Pose doorM = new Pose(15,65,Math.toRadians(180));
    private final Pose line1Pose = new Pose(13.5, 84, Math.toRadians(180));
    private final Pose line2Pose = new Pose(10, 59, Math.toRadians(180));
    private final Pose line3Pose = new Pose(10, 36, Math.toRadians(180));
    public final Pose endPose = new Pose(60,108,Math.toRadians(250));
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
        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(58.000, 51.000),
                                new Pose(55.000, 61.000),
                                line2Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line2Pose.getHeading())
                .build();
        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(63.000, 58.000),
                                new Pose(56.000, 60.000),
                                scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line2Pose.getHeading(),scorePose.getHeading())
                .build();
       /* doorPickup = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(58.000, 51.000),
                                new Pose(55.000, 61.000),
                                doorPose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),doorPose.getHeading())
                .build();
        doorMove = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, doorM)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(doorPose.getHeading(),doorM.getHeading())
                .build();
        scoreDoor = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(44.000, 65.000),
                                new Pose(54.000, 64.000),
                                scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(doorPose.getHeading(),scorePose.getHeading())
                .build(); */
        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(55.000, 79.000),
                                new Pose(48.000, 85.000),
                                line1Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line1Pose.getHeading())
                .build();
        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,
                                scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line1Pose.getHeading(),scorePose.getHeading())
                .build();
        grabPickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(62.000, 20.000),
                                new Pose(45.000, 37.000),
                                line3Pose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),line3Pose.getHeading())
                .build();
        scorePickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,
                                endPose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(line1Pose.getHeading(),endPose.getHeading(),0.1)
                .build();

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                r.pids=true;
                okp=true;
                okf=true;
                r.shoot=true;
                r.oks=true;
                nextPath();
                break;
            case 1:
                if(follower.getPose().getY()<105 && okf){
                    r.aiming=true;
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
                                    new BezierCurve(follower::getPose,
                                            new Pose(58.000, 51.000),
                                            new Pose(55.000, 61.000),
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
                    doorMove = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(28,59),
                                            doorM)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(doorPose.getHeading(),doorM.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.5) {
                        follower.followPath(doorMove, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(44.000, 65.000),
                                            new Pose(54.000, 64.000),
                                            scorePose)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(doorPose.getHeading(),scorePose.getHeading())
                            .build();
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
                break;
            case 6:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {
                    if (okp) {
                        pathTimer.resetTimer();
                        r.i.pornit = true;
                        okp = false;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(grabPickup1, true);
                        r.intake();
                        okp = true;
                        okf = true;
                        r.aiming = false;
                        nextPath();
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    r.aim=true;
                    r.aiming=true;
                    r.s.on();
                    okf=true;
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

                    doorPickup = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(58.000, 51.000),
                                            new Pose(55.000, 61.000),
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
            case 9:
                if(!follower.isBusy()) {
                    doorMove = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(28,59),
                                            doorM)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(doorPose.getHeading(),doorM.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1.5) {
                        follower.followPath(doorMove, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    scoreDoor = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierCurve(follower::getPose,
                                            new Pose(44.000, 65.000),
                                            new Pose(54.000, 64.000),
                                            scorePose)
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(doorPose.getHeading(),scorePose.getHeading())
                            .build();
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
                break;
            case 11:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {
                    if (okp) {
                        pathTimer.resetTimer();
                        r.i.pornit = true;
                        okp = false;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(grabPickup3, true);
                        r.intake();
                        okp = true;
                        okf = true;
                        r.aiming = false;
                        nextPath();
                    }
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    r.aim=true;
                    r.aiming=true;
                    r.s.on();
                    okf=true;
                    follower.followPath(scorePickup3,true);
                    nextPath();
                }
                break;
            case 13:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {
                    if (okp) {
                        pathTimer.resetTimer();
                        r.i.pornit = true;
                        okp = false;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        r.shooting=false;
                        r.s.off();
                        okp = true;
                        okf = true;
                        r.aiming = false;
                        nextPath();
                    }
                }
                break;

            case 14:
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


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,true,true,startPose);
        r.aInit();
        r.setShootTarget();
        //r.s.offset=-20;
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
