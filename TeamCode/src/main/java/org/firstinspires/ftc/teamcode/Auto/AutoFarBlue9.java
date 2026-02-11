package org.firstinspires.ftc.teamcode.Auto;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Auto Far Blue 9", group="Blue")
public class AutoFarBlue9 extends OpMode{

    public HubBulkRead bulk;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean okp,okf;
    private Robot r;
    private TelemetryManager t;

    private int pathState;
    private final Pose startPose = new Pose(56, 6, Math.toRadians(90));
    private final Pose scorePose = new Pose(52, 10, Math.toRadians(180));
    private final Pose humanPose = new Pose(12,8,Math.toRadians(180));
    private final Pose linePose = new Pose(12,36,Math.toRadians(180));
    public final Pose endPose = new Pose(40,14,Math.toRadians(180));

    private PathChain scorePreload,grabLine,scoreLine,humanGrab,humanScore,end;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();
        grabLine = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(58,41),
                                new Pose(42,35),
                                linePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(startPose.getHeading(),linePose.getHeading(),0.3)
                .build();
        scoreLine = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(linePose.getHeading(),scorePose.getHeading())
                .build();
        humanGrab = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, humanPose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),humanPose.getHeading())
                .build();
        humanScore = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(humanPose.getHeading(),scorePose.getHeading())
                .build();
        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, endPose)
                )
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(scorePose.getHeading(),endPose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(1);
                follower.followPath(scorePreload,true);
                r.pids=true;
                okp=true;
                okf=true;
                r.shoot=true;
                r.oks=true;
                r.tu.face(r.getShootTarget(),new Pose(scorePose.getX(),scorePose.getY(),Math.toRadians(90)));
                nextPath();
                break;
            case 1:
                if(!follower.isBusy()) {
                    if(okp){
                        r.aiming=true;
                        pathTimer.resetTimer();
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(grabLine, true);
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
                    r.tu.face(r.getShootTarget(),scorePose);
                    r.s.on();
                    okf=true;
                    follower.followPath(scoreLine,true);
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
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.aiming=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(humanGrab,true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    r.aim=true;
                    r.tu.face(r.getShootTarget(),scorePose);
                    r.s.on();
                    okf=true;
                    okp=true;
                    follower.followPath(humanScore,true);
                    nextPath();
                }
                break;
            case 5:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.aiming=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(humanGrab,true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    r.aim=true;
                    r.tu.face(r.getShootTarget(),scorePose);
                    r.s.on();
                    okf=true;
                    okp=true;
                    follower.followPath(humanScore,true);
                    nextPath();
                }
                break;
            case 7:
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
                        r.aiming=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(humanGrab,true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    r.aim=true;
                    r.tu.face(r.getShootTarget(),scorePose);
                    r.s.on();
                    okf=true;
                    okp=true;
                    follower.followPath(humanScore,true);
                    nextPath();
                }
                break;
            case 9:
                if(follower.getPose().getX()>20 && okf){
                    r.i.pornit=false;
                    r.s.latchdown();
                    okf=false;
                }
                if(!follower.isBusy()) {
                    if(okp){
                        pathTimer.resetTimer();
                        r.pids=true;
                        r.aiming=true;
                        r.i.pornit=true;
                        okp=false;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>1) {
                        follower.followPath(end,true);
                        r.shooting=false;
                        r.i.pornit=false;
                        r.aim=false;
                        okp=true;
                        r.aiming=false;
                        endPath();
                    }
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
        follower.usePredictiveBraking=true;
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,true,true,startPose);
        r.aInit();
        r.setShootTargetFar();
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
