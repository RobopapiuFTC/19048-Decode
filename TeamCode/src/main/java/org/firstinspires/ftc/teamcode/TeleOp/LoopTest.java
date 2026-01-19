package org.firstinspires.ftc.teamcode.TeleOp;

import android.provider.Settings;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Loop", group = "Bar")
public class LoopTest extends OpMode {
    TelemetryManager t;

    public Timer looptimer;
    public int loops;
    public double loopTime=0,lastLoop=0;
    @Override
    public void init() {
        t = PanelsTelemetry.INSTANCE.getTelemetry();
        looptimer = new Timer();
        looptimer.resetTimer();
    }

    @Override
    public void init_loop(){
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        loops++;

        if (loops > 10) {
            double now = looptimer.getElapsedTime();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }
        t.addData("Loop time", loopTime);
        t.addData("Loop time hz", 1000/loopTime);
        t.update(telemetry);
    }
}