package org.firstinspires.ftc.teamcode.Systems;

import android.service.controls.Control;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BangBang;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;

import com.ThermalEquilibrium.homeostasis.Parameters.BangBangParameters;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
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
    public Servo SVD,latch;
    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.001;
    private boolean activated = true;

    public double shootc=900,offset;

    public static double close = 1250;
    public static double far = 1400;

    public Shooter(HardwareMap hardwareMap, TelemetryManager telemetry){

        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        SVD=hardwareMap.get(Servo.class, "SVD");
        latch=hardwareMap.get(Servo.class,"latch");

        SD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SVD.setPosition(0.5);
        SD.setDirection(DcMotorSimple.Direction.REVERSE);
        latch.setPosition(0.7);
    }
    public void periodic(){
        if (activated)
            setPower((kV * getTarget()) + (kP * (getTarget() - getVelocity())) + kS);
    }
    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return SD.getVelocity();
    }
    public void setPower(double p) {
        SS.setPower(p);
        SD.setPower(p);
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
        setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+shootc+offset);
    }
    public void hoodfar(){
        SVD.setPosition(0.15);
    }
    public void hoodclose(){
        SVD.setPosition(0.15);
    }
    public void latchdown(){latch.setPosition(0.3);}
    public void latchup(){latch.setPosition(0.85);}

}
