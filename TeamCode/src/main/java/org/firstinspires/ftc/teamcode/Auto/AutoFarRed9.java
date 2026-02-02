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

@Autonomous(name="Auto Far Red 9", group="Red")
public class AutoFarRed9 extends OpMode{

    public HubBulkRead bulk;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private boolean okp;
    private Robot r;
    private TelemetryManager t;

    private int pathState;
    private  Pose startPose = new Pose(56, 9, Math.toRadians(90));
    private  Pose scorePose = new Pose(52, 12, Math.toRadians(180));
    private  Pose positionPose= new Pose(36,17,Math.toRadians(200));
    private  Pose line1Pose = new Pose(11, 6, Math.toRadians(200));
    private  Pose line2Pose = new Pose(11, 16, Math.toRadians(200));
    public  Pose endPose = new Pose(40,14,Math.toRadians(180));
    private  Pose linePose = new Pose(7.5,36,Math.toRadians(180));
    private PathChain scorePreload,position,grabLine,scoreLine,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3,end;
    public void buildPaths() {
        scorePreload = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose, scorePose)
                )
                .setLinearHeadingInterpolation(startPose.getHeading(),Math.toRadians(90))
                .build();

        position = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,positionPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(0))
                .build();
        grabLine = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(75,38).mirror(),
                                linePose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0),Math.toRadians(0))
                .build();
        scoreLine = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                follower::getPose,
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(linePose.getHeading(),scorePose.getHeading())
                .build();
        grabPickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(follower::getPose,
                                new Pose(7.000, 25).mirror(),
                                line1Pose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(),line1Pose.getHeading(),0.5)
                .build();

        scorePickup1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,scorePose)
                )
                .setLinearHeadingInterpolation(line1Pose.getHeading(),scorePose.getHeading())
                .build();

        grabPickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(8, 6).mirror(),
                                line2Pose
                        )
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(),line2Pose.getHeading())
                .build();

        scorePickup2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                follower::getPose,
                                new Pose(39, 12).mirror(),
                                scorePose
                        )
                )
                .setLinearHeadingInterpolation(line2Pose.getHeading(),scorePose.getHeading())
                .build();

        end = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(follower::getPose,endPose)
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
                nextPath();
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
                        r.shooting=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        follower.followPath(position, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(grabLine,true);
                    nextPath();
                }
                break;

            case 3:
                if(!follower.isBusy()) {
                    r.i.pornit=false;
                    follower.followPath(scoreLine,true);
                    nextPath();
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
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.shooting=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        follower.followPath(grabPickup1, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    r.i.pornit=false;
                    follower.followPath(scorePickup1,true);
                    nextPath();
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
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.shooting=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        follower.followPath(grabPickup2, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    r.i.pornit=false;
                    follower.followPath(scorePickup2, true);
                    nextPath();
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
                    if(pathTimer.getElapsedTimeSeconds()>0.5){
                        r.shooting=true;
                    }
                    if(pathTimer.getElapsedTimeSeconds()>3) {
                        follower.followPath(end, true);
                        r.intake();
                        okp=true;
                        r.aiming=false;
                        nextPath();
                    }

                }
                break;
            case 9:
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

        startPose = startPose.mirror();
        scorePose = scorePose.mirror();
        line1Pose = line1Pose.mirror();
        line2Pose = line2Pose.mirror();
        linePose = linePose.mirror();
        endPose = endPose.mirror();
        positionPose = positionPose.mirror();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        r = new Robot(hardwareMap,follower,t,gamepad1,gamepad2,false,true,startPose);
        r.aInit();
        r.setShootTargetFar();
        r.s.shootc=950;
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
