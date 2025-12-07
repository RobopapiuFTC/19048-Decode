package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    public static double error = 0, power = 0, manualPower = 0;
    public static double rpt = 0.0029919;

    public final DcMotorEx turret;
    private PIDFController p, s; // pidf controller for turret
    public static double t = 0;
    public static double pidfSwitch = 50; // target for turret
    public static double kp = 0.003, kf = 0.0, kd = 0.000, sp = .005, sf = 0, sd = 0.0001;

    public static boolean on = true, manual = false;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //turret.setDirection(DcMotor.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    private void setTurretTarget(double ticks) {
        t = ticks;
    }

    /** ticks */
    public double getTurretTarget() {
        return t;
    }

    /** ticks */
    private void incrementTurretTarget(double ticks) {
        t += ticks;
    }

    public double getTurret() {
        return turret.getCurrentPosition();
    }

    public void periodic() {
        if (on) {
            if (manual) {
                turret.setPower(manualPower);
                return;
            }
            p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
            s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
            error = getTurretTarget() - getTurret();
            if (error > pidfSwitch) {
                p.updateError(error);
                power = p.run();
            } else {
                s.updateError(error);
                power = s.run();
            }

            turret.setPower(power);
        } else {
            turret.setPower(0);
        }
    }

    public void manual(double power) {
        manual = true;
        manualPower = power;
    }

    public void automatic() {
        manual = false;
    }

    public void on() {
        on = true;
    }

    public void off() {
        on = false;
    }

    /** Return yaw in radians */
    public double getYaw() {
        return normalizeAngle(getTurret() * rpt);
    }

    public void setYaw(double radians) {
        radians = normalizeAngle(radians);
        setTurretTarget(radians/rpt);
    }

    public void addYaw(double radians) {
        setYaw(getYaw() + radians);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(angleToTargetFromCenter - robotPose.getHeading());
        setYaw(robotAngleDiff);
    }

    public void resetTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTurretTarget(0);
    }


    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }

    public double getError() {
        return error;
    }

    public boolean isReady() {
        return Math.abs(getError()) < 30;
    }
}