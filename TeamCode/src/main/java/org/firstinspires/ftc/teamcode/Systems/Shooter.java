package org.firstinspires.ftc.teamcode.Systems;

import android.service.controls.Control;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.util.Timer;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

public class Shooter {
    public DcMotorEx SS,SD;
    public Servo SVS,SVD;
    public Telemetry telemetry;
    public boolean pornit=false,yea=false;
    public double p,i,d;
    public double output,target;
    public ControlSystem pid;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){

        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        //SVS=hardwareMap.get(Servo.class, "SVS");
        SVD=hardwareMap.get(Servo.class, "SVD");

        SS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SVD.setPosition(0.5);
        SD.setDirection(DcMotorSimple.Direction.REVERSE);

        pid = ControlSystem.builder()
                .velPid(p,i,d)
                .build();
    }
    public void periodic(){
        run();
    }
    public void run(){
        output = pid.calculate(
               new KineticState(SS.getCurrentPosition(), target,20)
        );
        SS.setPower(output);
        SD.setPower(output);
    }
    public void hoodfar(){
        SVD.setPosition(0.75);
    }
}
