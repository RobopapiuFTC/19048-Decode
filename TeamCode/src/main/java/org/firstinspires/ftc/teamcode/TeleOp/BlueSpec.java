package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "BlueSpec", group = "...Sigma")
public class BlueSpec extends OpMode {

    Robot r;


    @Override
    public void init() {
        r = new Robot(hardwareMap, telemetry, gamepad1 , gamepad2,true,true);
        r.tInit();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        r.dualControls();
        r.tPeriodic();
    }
}