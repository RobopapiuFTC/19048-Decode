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
    public DcMotorEx turret;
    public double power,pos,targett;
    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.001;
    public static double p = 0.01, i = 0, d = 0.00000000000005, f = 0.05;
    public PIDController pid;
    private boolean activated = true;

    public static double close = 1250;
    public static double far = 1400;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){

        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        //SVS=hardwareMap.get(Servo.class, "SVS");
        SVD=hardwareMap.get(Servo.class, "SVD");
        latch=hardwareMap.get(Servo.class,"latch");

        SD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SVD.setPosition(0.5);
        SD.setDirection(DcMotorSimple.Direction.REVERSE);
        latch.setPosition(0.3);

        pid = new PIDController(p , i , d);

        turret=hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void periodic(){
        if (activated)
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
        runt();
    }
    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return SD.getVelocity();
    }
    public void setPower(double p) {
        SS.setPower(-p);
        SD.setPower(-p);
    }
    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public boolean isActivated() {
        return activated;
    }
    public void far() {
        setTarget(far);
        on();
    }
    public void close() {
        setTarget(close);
        on();
    }
    public void setTarget(double velocity) {
        t = velocity;
    }
    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }
    public void forDistance(double distance) {
        //setTarget((6.13992 * distance) + 858.51272);
        setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+948.97358);
    }
    public void hoodfar(){
        SVD.setPosition(0.2);
    }
    public void hoodclose(){
        SVD.setPosition(0.25);
    }
    public void latchdown(){latch.setPosition(0.9);}
    public void latchup(){latch.setPosition(0.3);}
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

}
