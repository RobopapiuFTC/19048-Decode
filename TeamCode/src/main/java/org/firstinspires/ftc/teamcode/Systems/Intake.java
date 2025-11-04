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
public class Intake {
    public DcMotorEx intake;
    public Servo SVS,SVD;
    public Telemetry telemetry;
    public boolean pornit=false;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry){

        intake=hardwareMap.get(DcMotorEx.class, "i");

    }
    public void periodic(){
        run();
    }

    public void run(){
        if(pornit){
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }
    }
}
