package org.firstinspires.ftc.teamcode.Systems;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.aim;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.auto;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Systems.Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret {
    public Camera c;
    public static double error = 0, power = 0, manualPower = 0;
    public static double rpt = 6.28319/1900;
    public static double gear = 1900/360,target;
    public double tti,tpc=0;
    public static double offset=0;
    public boolean a;
    public final Servo t1,t2;

   // public final DcMotorEx turret;
    private PIDFController p, s;
    public static double t = 0, cameraerror=0;
    public static double pidfSwitch = 30;
    public static double kp = 0.01, kf = 0.0, kd = 0.000, sp = .013, sf = 0, sd = 0.0001;
    //public static double kp = 0.003, kf = 0.0, kd = 0.000, sp = .005, sf = 0, sd = 0.0001;
    public Timer cameraTimer;
    public boolean okt=false;

    public static boolean on = true, manual = false;

    public Turret(HardwareMap hardwareMap, TelemetryManager telemetry, boolean a) {
        /*turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); */
        t1=hardwareMap.get(Servo.class, "t1");
        t2=hardwareMap.get(Servo.class, "t2");
        t1.setPosition(0);
        t2.setPosition(0);
        this.a=a;
        c = new Camera(hardwareMap,telemetry,a);
        c.start();

        p = new PIDFController(new PIDFCoefficients(kp, 0, kd, kf));
        s = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
        cameraTimer = new Timer();
    }

    public void setTurretTarget(double ticks) {
        if(ticks>=1900)ticks=ticks-1900;
        if(ticks<0)ticks=ticks+1900;
        t=ticks;
    }
    public double getTurretTarget() {
        return t;
    }

    public double getTurret() {
       // return turret.getCurrentPosition();
        return 0;
    }

    public void periodic() {
       /* if (on) {
            if(!auto) {
                if (okt) {
                    if (getTurret() >= getTurretTarget() - 3 && getTurret() <= getTurretTarget() + 3) {
                        c.detect();
                        if (c.result != null && c.result.isValid()) {
                            if (a) {
                                tpc = -c.tx * 5.27777777778;
                                cameraerror = cameraerror + tpc;

                            } else {
                                tpc = c.tx * 5.27777777778;
                                cameraerror = cameraerror + tpc;
                            }
                            c.result = null;
                            okt = false;
                        }
                    }
                }
            }
            if (manual) {
                turret.setPower(manualPower);
                return;
            }
            p.setCoefficients(new PIDFCoefficients(kp, 0, kd, kf));
            s.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));
            error = getTurretTarget() - getTurret();

            s.updateError(error);
            s.updateFeedForwardInput(Math.signum(error));
            power = s.run();
            *//* if (error > pidfSwitch) {
                p.updateError(error);
                p.updateFeedForwardInput(Math.signum(error));
                power = p.run();
            } else {
                s.updateError(error);
                power = s.run();
            }

            turret.setPower(power);
        } else {
            turret.setPower(0);
        }*/
        if(on){
            target=clamp(t/gear/355,0,1);
            t1.setPosition(target);
            t2.setPosition(target);
        }
        else{
            t1.setPosition(0);
            t2.setPosition(0);
        }

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
        setTurretTarget(radians/rpt+cameraerror);
    }

    public void face(Pose targetPose, Pose robotPose) {
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(+angleToTargetFromCenter - robotPose.getHeading()+tti);
        setYaw(robotAngleDiff+offset);
    }
    public double Yaw(Pose targetPose, Pose robotPose){
        double angleToTargetFromCenter = Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX());
        double robotAngleDiff = normalizeAngle(+angleToTargetFromCenter - robotPose.getHeading()+tti);
        return robotAngleDiff;
    }

    public void resetTurret() {
       // turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setTurretTarget(0);
    }


    public static double normalizeAngle(double angleRadians) {
        double angle = angleRadians % (Math.PI * 2D);
        if (angle <= -Math.PI) angle += Math.PI * 2D;
        if (angle > Math.PI) angle -= Math.PI * 2D;
        return angle;
    }
}