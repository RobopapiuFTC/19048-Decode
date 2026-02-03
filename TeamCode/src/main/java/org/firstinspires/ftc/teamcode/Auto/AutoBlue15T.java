package org.firstinspires.ftc.teamcode.Auto;
import android.graphics.Point;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto 15 Blue Tangent", group="Blue")
public class AutoBlue15T extends OpMode{

    public HubBulkRead bulk;
    private TelemetryManager t;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Robot r;
    private boolean okp,okf;

    private int pathState;
    private  Pose startPose = new Pose(20, 125, Math.toRadians(234));
    private  Pose scorePose = new Pose(50, 84, Math.toRadians(180));
    private  Pose doorPose = new Pose(24,60,Math.toRadians(180));
    private  Pose doorM = new Pose(13,62,Math.toRadians(153));
    private  Pose line1Pose = new Pose(13, 84, Math.toRadians(180));
    private  Pose line2Pose = new Pose(8, 59, Math.toRadians(180));
    private  Pose line3Pose = new Pose(8, 35, Math.toRadians(180));
    public  Pose endPose = new Pose(59,106,Math.toRadians(180));
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
                        new BezierCurve(
                                follower::getPose,
                                new Pose(52, 53),
                                new Pose(56, 62),
                                line2Pose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(40, 64),
                                scorePose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation().setReversed()
                .build();

        doorPickup = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(52, 53),
                                new Pose(56, 62),
                                doorPose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation()
                .build();
        scoreDoor = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(36,57),
                                scorePose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation().setReversed()
                .build();
        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                line1Pose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation()
                .build();
        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                scorePose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation().setReversed()
                .build();
         grabPickup3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(58,27),
                                new Pose(53,38),
                                line3Pose
                        )
                )
                .setBrakingStrength(2)
                .setTangentHeadingInterpolation()
                .build();
         scorePickup3 = follower
                 .pathBuilder()
                 .addPath(
                         new BezierCurve(
                                 follower::getPose,
                                 new Pose(41,36),
                                 new Pose(52,28),
                                 endPose
                         )
                 )
                 .setBrakingStrength(2)
                 .setTangentHeadingInterpolation().setReversed()
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
                                    new BezierCurve(
                                            follower::getPose,
                                            new Pose(52, 53),
                                            new Pose(56, 62),
                                            doorPose
                                    )
                            )
                            .setBrakingStrength(2)
                            .setTangentHeadingInterpolation()
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
                                    new BezierCurve(
                                            follower::getPose,
                                            doorM
                                    )
                            )
                            .setBrakingStrength(2)
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), doorM.getHeading())
                            .build();
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        okp=false;

                    }
                    if(pathTimer.getElapsedTimeSeconds()>0) {
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
                                    new BezierCurve(
                                            follower::getPose,
                                            new Pose(36,57),
                                            scorePose
                                    )
                            )
                            .setBrakingStrength(2)
                            .setTangentHeadingInterpolation().setReversed()
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
                    r.aim=true;
                    r.aiming=true;
                    r.s.on();
                    okf=true;
                    okp=true;
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
                    r.aim=true;
                    r.aiming=true;
                    r.s.on();
                    okf=true;
                    okp=true;
                    follower.followPath(scorePickup3,true);
                    nextPath();
                }
                break;
            case 10:
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
            case 11:
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
