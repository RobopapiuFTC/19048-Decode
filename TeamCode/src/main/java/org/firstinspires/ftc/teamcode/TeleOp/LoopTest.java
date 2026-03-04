package org.firstinspires.ftc.teamcode.TeleOp;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.HubBulkRead;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Loop", group = "Bar")
public class LoopTest extends OpMode {
    Robot r;
    private Follower follower;
    TelemetryManager t;
    public static Pose startingPose = new Pose(23, 128, Math.toRadians(234));
    public static Pose parkPose = new Pose(111,40,Math.toRadians(270));
    public static Pose relocalization = new Pose(132,8,Math.toRadians(0));
    public static Pose relocalization2 = new Pose(33,137.5,Math.toRadians(270));
    public static PathChain park;
    public HubBulkRead bulk;

    public Timer looptimer;
    public int loops;
    public double loopTime=0,lastLoop=0;
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        bulk = new HubBulkRead(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        r = new Robot(hardwareMap,follower, t, gamepad1 , gamepad2,true,false,startingPose);
        r.tInit();
        r.setRelocalization(relocalization,relocalization2);
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start() {
        r.tStart();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        r.tu.on();
        r.tu.automatic();
        if(gamepad1.dpad_down){
            r.tu.setYaw(Math.toRadians(180));
        }
        if(gamepad1.dpad_right)r.tu.setYaw(Math.toRadians(270));
        if(gamepad1.dpad_left)r.tu.setYaw(Math.toRadians(90));
        if(gamepad1.dpad_up)r.tu.setYaw(0);
        if(gamepad1.y)r.tu.setYaw(Math.toRadians(355));
        r.tu.periodic();
        t.addData("Turret Position", r.tu.target);
        t.addData("Turret Angle",Math.toDegrees(r.tu.t));
        t.addData("Velocity: ", r.s.getVelocity());
        t.addData("Dist: ", r.dist);
        t.addData("Shooter offset: ", r.s.offset);
        t.addData("Follower Pose: ", r.f.getPose().toString());
        t.addData("Loop time: ", r.getLoopTimeMs());
        t.addData("Loop time hz: ", r.getLoopTimeHz());
        t.update(telemetry);
    }
}