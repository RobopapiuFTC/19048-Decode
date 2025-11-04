package org.firstinspires.ftc.teamcode.Systems;

import android.service.controls.Control;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BangBang;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.BangBangParameters;
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
    public double target;
    public boolean pornit=false,yea=false;
    public double p,i,d;
    double maxOutput = 1;
    double tolerance = 3;
    double command,estimate;
    double Q = 0.3; // High values put more emphasis on the sensor.
    double R = 3; // High Values put more emphasis on regression.
    int N = 3; // The number of estimates in the past we perform regression on.
    BangBangParameters parameters = new BangBangParameters(maxOutput,tolerance);
    KalmanFilter filter = new KalmanFilter(Q,R,N);
    BangBang controller = new BangBang(parameters);
    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){

        SS=hardwareMap.get(DcMotorEx.class, "SS");
        SD=hardwareMap.get(DcMotorEx.class, "SD");
        //SVS=hardwareMap.get(Servo.class, "SVS");
        SVD=hardwareMap.get(Servo.class, "SVD");

        SS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SVD.setPosition(0.5);
        SD.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void periodic(){
        run();
    }
    public void run(){
        command = controller.calculate(target, SD.getVelocity());
        estimate = filter.estimate(command);
        SD.setPower(command);
        SS.setPower(command);
    }
    public void hoodfar(){
        SVD.setPosition(0.75);
    }
}
