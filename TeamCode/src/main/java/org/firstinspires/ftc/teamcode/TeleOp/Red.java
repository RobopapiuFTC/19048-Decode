package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Hardware.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Red", group = "...Sigma")
public class Red extends OpMode {

    Robot r;
    private Follower follower;
    TelemetryManager t;
    public static Pose startingPose = new Pose(20, 125, Math.toRadians(234)).mirror();
    public static Pose parkPose = new Pose(111,40,Math.toRadians(270)).mirror();
    public static PathChain park;
    public HubBulkRead bulk;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        bulk = new HubBulkRead(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        r = new Robot(hardwareMap,follower, t, gamepad1 , gamepad2,false,false,startingPose);
        r.tInit();
    }

    @Override
    public void init_loop(){
        if(gamepad1.x)r.tu.resetTurret();
        if(gamepad1.y){
            follower.setPose(startingPose);
        }
    }

    @Override
    public void start() {
        r.tStart();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        bulk.clearCache(HubBulkRead.Hubs.ALL);
        follower.update();
        if(r.slowmode){
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y*0.5,
                    -gamepad1.left_stick_x*0.5,
                    -gamepad1.right_stick_x*0.5,
                    true
            );
        }
        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );
        if(gamepad1.share && gamepad1.left_trigger>0.3){
            park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(follower.getPose(), parkPose)
                    )
                    .setBrakingStrength(2)
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(),parkPose.getHeading(),0.2)
                    .build();
            follower.followPath(park,true);
        }
        else if(gamepad1.share)follower.startTeleopDrive();
        r.dualControls();
        r.tPeriodic();
        t.addData("Velocity: ", r.s.getVelocity());
        t.addData("Dist: ", r.dist);
        t.addData("Shooter offset: ", r.s.offset);
        t.addData("Camera Error", r.tu.cameraerror);
        t.addData("Turret Ticks: ", r.tu.getTurret());
        t.addData("Follower Pose: ", r.f.getPose().toString());
        t.addData("Loop time: ", r.getLoopTimeMs());
        t.addData("Loop time hz: ", r.getLoopTimeHz());
        t.addData("Intake Velocity: ", r.i.getVelocity());
        t.update(telemetry);
    }
    @Override
    public void stop(){
        r.stop();
    }
}