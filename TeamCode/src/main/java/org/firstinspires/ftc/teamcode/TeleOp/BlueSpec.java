package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "BlueSpec", group = "...Sigma")
public class BlueSpec extends OpMode {

    Robot r;
    private Follower follower;
    TelemetryManager t;
    public static Pose startingPose = new Pose(72,135,Math.toRadians(90));


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        r = new Robot(hardwareMap,follower, t, gamepad1 , gamepad2,true,true,startingPose);
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
    }

    @Override
    public void loop() {
        follower.update();
        r.dualControls();
        r.tPeriodic();
        t.addData("Velocity: ", r.s.getVelocity());
        t.addData("Dist: ", r.dist);
        t.addData("Turret Ticks", r.tu.getTurret());
        t.addData("Follower Pose", r.f.getPose().toString());
        t.update(telemetry);
    }
}