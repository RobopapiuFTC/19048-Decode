package org.firstinspires.ftc.teamcode.Hardware;
import static java.lang.Math.asin;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.Systems.Shooter;
import org.firstinspires.ftc.teamcode.Systems.Movement;
import org.firstinspires.ftc.teamcode.Systems.Intake;
import org.firstinspires.ftc.teamcode.Systems.Camera;
import org.firstinspires.ftc.teamcode.Systems.Turret;

public class Robot {
    private HardwareMap h;
    private TelemetryManager t;
    public Shooter s;
    public Intake i;
    private Movement m;
    public Turret tu;
    private Camera c;
    public double dist;
    public static Pose startingPose = new Pose(72,135,Math.toRadians(90));
    public static Pose shootp = new Pose(0 ,144,0);
    public static Pose endPose;
    public Gamepad g1,g2;
    public Follower f;
    public boolean a,shoot,oks,aim,auto,intake,oki,pids=false,slowmode=false,aima=true;
    public double c1,c2,ipoten,unghi,ticksneeded;
    public Timer iTimer,rTimer,rsTimer,sTimer,oTimer;
    public static int offset=0;
    double ticksfor360 = 1900;
    double ticksperdegree = ticksfor360 / 360;
    public Robot(HardwareMap h, Follower f, TelemetryManager t, Gamepad g1, Gamepad g2, boolean blue, boolean auto) {
        this.h = h;
        this.t = t;
        this.f = f;
        this.g1 = g1;
        this.g2 = g2;
        this.a = blue;
        this.auto = auto;
        s=new Shooter(this.h,this.t);
        m=new Movement(this.h, this.t);
        i=new Intake(this.h,this.t);
        tu=new Turret(this.h,this.t);
       // c=new Camera(this.h,this.t,blue);

        iTimer = new Timer();
        rTimer = new Timer();
        rsTimer = new Timer();
        sTimer = new Timer();
        oTimer = new Timer();
    }
    public void stop() {
        endPose = f.getPose();
    }

    public void tPeriodic() {
        f.update();
        setShootTarget();
        sequenceshoot();
        sequenceintake();
        if(aim)turret();
        else{
            tu.set(0);
        }
        m.periodic(g1);
        i.periodic();
        s.periodic();
        tu.periodic();
    }
    public void tStart(){
        setShootTarget();
    }
    public void tInit() {

    }
    public void aPeriodic(){
        sequenceshoot();
        sequenceintake();
        setShootTarget();
        if(aim)turret();
        else{
            tu.set(0);
        }
        if(pids){
            s.periodic();
            tu.periodic();
        }
        i.periodic();
        t.update();
    }
    public void aInit(){
        tu.reset();
        setShootTarget();
    }
    public void dualControls(){
        if(g1.y){
            if(rTimer.getElapsedTimeSeconds()>0.3){
               intake();
                rTimer.resetTimer();
            }
        }
        if(g1.x){
            if(rTimer.getElapsedTimeSeconds()>0.3){
                shooter();
                rTimer.resetTimer();
            }
        }
        if(g1.dpad_left){
            if(oTimer.getElapsedTimeSeconds()>0.5){
                tu.add(0.0872665);
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_right){
            if(oTimer.getElapsedTimeSeconds()>0.5){
                tu.add(-0.0872665);
                oTimer.resetTimer();
            }
        }
        if(g1.a){
            i.pornit=true;
        }
    }
    public void intake(){
        intake=true;
        oki=true;
    }
    public void shooter(){
        shoot=true;
        oks=true;
    }
    public void sequenceintake(){
        if(intake) {
            if (oki) {
                iTimer.resetTimer();
                oki = false;
            }
            if (iTimer.getElapsedTimeSeconds() < 0.4) {
                aim = false;
                s.latchup();
            }
            if (iTimer.getElapsedTimeSeconds() > 0.4 && iTimer.getElapsedTimeSeconds() < 1) {
                i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                i.pornit = true;
                intake=false;
                oki=false;
            }
        }
    }
    public void sequenceshoot(){
        if(shoot) {
            if(oks){
                aim=true;
                sTimer.resetTimer();
                oks=false;
            }
            if(sTimer.getElapsedTimeSeconds()<0.2){
                if(!auto) {
                    i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    i.pornit = true;
                }
            }
            if(sTimer.getElapsedTimeSeconds()>0.2 && sTimer.getElapsedTimeSeconds()<1){
                i.pornit=false;
                i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                s.latchdown();
                shoot=false;
                oks=false;
            }
        }
    }
    public void turret() {
            if (f.getPose().getY() > 40) {
                s.hoodclose();
                dist = shootp.distanceFrom(f.getPose());
                s.forDistance(dist);
            } else {
                s.hoodfar();
                dist = shootp.distanceFrom(f.getPose());
                s.forDistance(dist);
            }
            tu.face(getShootTarget(),f.getPose());
            tu.automatic();
    }
    public void setShootTarget() {
        if (a)
            shootp = new Pose(0, 144, 0);
        else
            shootp = shootp.mirror();
    }
    public Pose getShootTarget() {
        return shootp;
    }

}
