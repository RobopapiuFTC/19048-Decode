package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.util.Timer;
public class Shooter {
    public DcMotorEx SS,SD;
    public Servo SVS,SVD;
    public Telemetry telemetry;
    public boolean pornit=false;
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){

        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        //SVS=hardwareMap.get(Servo.class, "SVS");
        SVD=hardwareMap.get(Servo.class, "SVD");

        SVD.setPosition(0.5);
        SD.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void periodic(){
        run();
    }

    public void run(){
        if(pornit){
            SS.setPower(0.85);
            SD.setPower(0.85);
        }else{
            SS.setPower(0);
            SD.setPower(0);
        }
    }
    public void hoodfar(){
        SVD.setPosition(0.75);
    }
}
