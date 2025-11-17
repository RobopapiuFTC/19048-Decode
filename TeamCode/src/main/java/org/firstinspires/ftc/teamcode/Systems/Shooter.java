package org.firstinspires.ftc.teamcode.Systems;

import android.service.controls.Control;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BangBang;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.arcrobotics.ftclib.controller.PIDController;

import com.ThermalEquilibrium.homeostasis.Parameters.BangBangParameters;
import com.bylazar.configurables.annotations.Configurable;
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
@Configurable
public class Shooter {
    public DcMotorEx SS,SD;
    public Servo SVS,SVD,latch;
    public Telemetry telemetry;
    public double target;
    public double power;
    public double pos;
    public boolean pornit=false,yea=false;
    double maxOutput = 1;
    double tolerance = 3;
    double command,estimate,putere;
    double Q = 0.3; // High values put more emphasis on the sensor.
    double R = 3; // High Values put more emphasis on regression.
    int N = 3; // The number of estimates in the past we perform regression on.
    public DcMotorEx turret;
    public PIDController pid;
    public static double p = 0.01, i = 0, d = 0.00000000000005, f = 0.05;
    public double targett;
    BangBangParameters parameters = new BangBangParameters(maxOutput,tolerance);
    KalmanFilter filter = new KalmanFilter(Q,R,N);
    BangBang controller = new BangBang(parameters);
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){

        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        //SVS=hardwareMap.get(Servo.class, "SVS");
        SVD=hardwareMap.get(Servo.class, "SVD");
        latch=hardwareMap.get(Servo.class,"latch");

        SD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SVD.setPosition(0.5);
        SD.setDirection(DcMotorSimple.Direction.REVERSE);
        latch.setPosition(0.25);

        pid = new PIDController(p , i , d);

        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void periodic(){
        run();
        runt();
    }
    public void run(){
        command = controller.calculate(target, SD.getVelocity());
        SD.setPower(command);
        SS.setPower(command);
    }
    public int getPos() {
        pos = turret.getCurrentPosition();
        return turret.getCurrentPosition();
    }

    public void runt(){
        pid.setPID(p,i,d);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        double pid_output = pid.calculate(getPos(), targett);
        power = pid_output + f;
        turret.setPower(pid_output);
    }
    public void hoodfar(){
        SVD.setPosition(0);
    }
    public void hoodclose(){
        SVD.setPosition(0.4);
    }
    public void latchdown(){
        latch.setPosition(0.65);
    }
    public void latchup(){
        latch.setPosition(0.35);
    }
}
