package org.firstinspires.ftc.teamcode.Systems;

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
    public DcMotorEx intake;
    public RevColorSensorV3 sensor;
    public Servo SVS,SVD;
    public Telemetry telemetry;
    public NormalizedRGBA colors;
    public boolean pornit=false,looping=false;
    public int pos,f1,f2;
    public double distance;
    public boolean[] full;
    public Timer loopTimer,readTimer;
    public Intake(HardwareMap hardwareMap, TelemetryManager telemetry){

        intake=hardwareMap.get(DcMotorEx.class, "i");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensor = hardwareMap.get(RevColorSensorV3.class,"color");
        full = new boolean[]{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};

        loopTimer=new Timer();
        readTimer=new Timer();
    }
    public void periodic(){
        run();
        //isFull();
    }

    public void run(){
        if(pornit){
            intake.setPower(1);
        }else{
            intake.setPower(0);
        }
    }
    public void isFull(){
        if(!pornit)return;
            if(loopTimer.getElapsedTimeSeconds()<0.6) {
                if(readTimer.getElapsedTimeSeconds()<0.05) {
                    if(looping) {
                        distance = sensor.getDistance(DistanceUnit.MM);
                        if (distance < 25) {
                            full[pos]=true;
                        } else {
                            full[pos]=false;
                        }
                        looping=false;
                    }
                }
                else{
                    looping=true;
                    pos++;
                    readTimer.resetTimer();
                }
            }
            else{
                for(int i=1; i<pos; i++) {
                    if(full[i])f1++;
                    else f2++;
                    full[i]=false;
                }
                pos=1;
                looping=true;
                loopTimer.resetTimer();
                if(f1>f2){
                    pornit=false;
                    f1=0;
                    f2=0;
                }
            }

    }

    public double getVelocity(){
        return intake.getVelocity();
    }
}