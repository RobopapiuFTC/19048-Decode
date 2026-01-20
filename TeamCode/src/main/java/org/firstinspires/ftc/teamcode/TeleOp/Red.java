package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Red", group = "...Sigma")
public class Red extends OpMode {

    Robot r;
    private Follower follower;
    TelemetryManager t;
    public static Pose startingPose = new Pose(20, 125, Math.toRadians(234)).mirror();


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
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

        r.dualControls();
        r.tPeriodic();
        t.addData("Velocity: ", r.s.getVelocity());
        t.addData("Dist: ", r.dist);
        t.addData("Turret Ticks", r.tu.getTurret());
        t.addData("Follower Pose", r.f.getPose().toString());
        t.addData("Loop time", r.getLoopTimeMs());
        t.addData("Loop time hz", r.getLoopTimeHz());
        t.addData("Intake Velocity", r.i.getVelocity());
        t.update(telemetry);
    }
}