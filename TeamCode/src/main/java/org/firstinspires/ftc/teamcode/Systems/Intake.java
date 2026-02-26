package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Robot.rumble;

import android.graphics.Color;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.util.Timer;
public class Intake {
    public DcMotorEx intake,transfer;
    public RevColorSensorV3 s1,s2,s3;
    public Telemetry telemetry;
    public boolean pornit=false,looping=false,oki=true,third=false,second=false;
    public static double t = 0;
    public double d1,d2,d3;
    public Timer loopTimer,readTimer;
    public Intake(HardwareMap hardwareMap, TelemetryManager telemetry){

        intake=hardwareMap.get(DcMotorEx.class, "i");
        transfer=hardwareMap.get(DcMotorEx.class,"t");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        s1 = hardwareMap.get(RevColorSensorV3.class,"s1");
        s2=hardwareMap.get(RevColorSensorV3.class,"s2");
        s3=hardwareMap.get(RevColorSensorV3.class,"s3");

        loopTimer=new Timer();
        readTimer=new Timer();
    }
    public void periodic(){
        run();
    }

    public double getTarget() {
        return t;
    }
    public void setPower(double p) {
        transfer.setPower(p);
    }

    public void setTarget(double velocity) {
        t = velocity;
    }
    public void run(){
        if(pornit){
                transfer.setPower(1);
                intake.setPower(1);
        }else{
           transfer.setPower(0);
           intake.setPower(0);
        }
    }
    public void isFull(){
        if(!looping)return;
        d1 = s1.getDistance(DistanceUnit.MM);
        d2 = s2.getDistance(DistanceUnit.MM);
        d3 = s3.getDistance(DistanceUnit.MM);
        if(d3<30)third=true;
        if(third && d2<10)second=true;
        if(third && second && d1<13){
            if(oki) {
                readTimer.resetTimer();
                oki=false;
            }
        }
        if(readTimer.getElapsedTimeSeconds()>0.4 && !oki) {
            pornit = false;
            looping = false;
            rumble = true;
        }
    }

    public double getVelocity(){
        return transfer.getVelocity();
    }
}