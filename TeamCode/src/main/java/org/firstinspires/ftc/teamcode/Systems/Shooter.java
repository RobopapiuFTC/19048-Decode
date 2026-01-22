package org.firstinspires.ftc.teamcode.Systems;

import android.service.controls.Control;

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
import org.firstinspires.ftc.teamcode.Util.ShotSample;

import com.pedropathing.util.Timer;

import java.util.Arrays;
import java.util.List;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
@Configurable
public class Shooter {
    public DcMotorEx SS,SD;
    public Servo SVD,latch;
    private double t = 0;
    public static double kS = 0.08, kV = 0.00039, kP = 0.001;
    private boolean activated = true;

    public double shootn=1000,shootc=1000,offset;
    public static double hood,angle=0.0006;

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(50, 1280, 0.97),
            new ShotSample(55, 1290, 0.97),
            new ShotSample(60, 1300, 0.97),
            new ShotSample(65, 1310, 0.97),
            new ShotSample(70, 1320, 0.85),
            new ShotSample(75, 1330, 0.85),
            new ShotSample(80, 1340, 0.8),
            new ShotSample(85, 1360, 0.8),
            new ShotSample(90, 1380, 0.76),
            new ShotSample(95, 1400, 0.76),
            new ShotSample(100, 1420, 0.58),
            new ShotSample(105, 1440, 0.58),
            new ShotSample(110, 1460, 0.55),
            new ShotSample(115, 1500, 0.55),
            new ShotSample(120, 1540, 0.5),
            new ShotSample(125, 1560, 0.5),
            new ShotSample(130, 1580, 0.5),
            new ShotSample(140, 1620, 0.35),
            new ShotSample(150, 1680, 0.35),
            new ShotSample(160, 1700, 0.3)
    );

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
    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
    private ShotSample lookupShot(double d) {

        ShotSample before = null;
        ShotSample after  = null;

        // find closest sample <= d  and closest sample >= d
        for (ShotSample s : samples) {
            if (s.distance <= d) {
                if (before == null || s.distance > before.distance) {
                    before = s;
                }
            }
            if (s.distance >= d) {
                if (after == null || s.distance < after.distance) {
                    after = s;
                }
            }
        }

        if (before == null) return after;
        if (after == null) return before;
        if (before.distance == after.distance) {
            return before;
        }

        double t = (d - before.distance) / (after.distance - before.distance);

        double power = lerp(before.power, after.power, t);

        return new ShotSample(d, power, angle);
    }

    public void on() {
        activated = true;
    }
    public void setTarget(double velocity) {
        t = velocity;
    }
    public boolean atTarget() {
        return Math.abs((getTarget()- getVelocity())) < 50;
    }
    public void forDistance(double distance) {
        //setTarget((0.00180088*Math.pow(distance, 2))+(4.14265*distance)+shootc+offset);
        ShotSample shoot = lookupShot(distance);
        setTarget(shoot.power + offset);
    }
    public void hoodfar(){
        SVD.setPosition(0.15);
    }
    public void hoodclose(){
        SVD.setPosition(0.15);
    }
    public void latchdown(){latch.setPosition(0.24);}
    public void latchup(){latch.setPosition(0.75);}

}
