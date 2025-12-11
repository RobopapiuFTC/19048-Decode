package org.firstinspires.ftc.teamcode.Hardware;
import static java.lang.Math.asin;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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
    public double dist;
    public static Pose shootp = new Pose(0 ,144,0);
    public static Pose parkPose,endPose,startingPose=new Pose(72,135,90);
    public Gamepad g1,g2;
    public Follower f;
    public boolean a,shoot,oks,aim,auto,intake,oki,pids=false,aima=true;
    public Timer iTimer,rTimer,rsTimer,sTimer,oTimer;
    public PathChain park;
    public Robot(HardwareMap h, Follower f, TelemetryManager t, Gamepad g1, Gamepad g2, boolean blue, boolean auto,Pose startingPose) {
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
        tu=new Turret(this.h,this.t,this.a);

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
        setShootTarget();
        sequenceshoot();
        sequenceintake();
        if(aim)turret();
        else{
            tu.setYaw(0);
        }
        m.periodic(g1);
        i.periodic();
        s.periodic();
        tu.periodic();
    }
    public void tStart(){
        setShootTarget();
        setTurretOffset();
    }
    public void tInit() {
        if(endPose==null){
            f.setPose(startingPose);
        }
        else f.setPose(endPose);
        tu.resetTurret();
        tu.c.start();
        setTurretOffset();
    }
    public void aPeriodic(){
        sequenceshoot();
        sequenceintake();
        setShootTarget();
        if(aim)turret();
        else{
            tu.setYaw(0);
        }
        if(pids){
            s.periodic();
            tu.periodic();
        }
        i.periodic();
    }
    public void aInit(){
        tu.resetTurret();
        tu.c.start();
        setShootTarget();
        setTurretOffset();
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
        if(g1.b){
            f.setPose(startingPose);
        }
        if(g1.left_bumper){
            park = f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    parkPose
                            )
                    )
                    .setLinearHeadingInterpolation(f.getPose().getHeading(),parkPose.getHeading())
                    .build();
            f.followPath(park);
        }
        if(g1.dpad_left){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                tu.offset=tu.offset+0.0872665;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_right){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                tu.offset=tu.offset-0.0872665;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_up){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                s.offset=s.offset+20;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_down){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                s.offset=s.offset-20;
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
                s.shootc=880;
                dist = shootp.distanceFrom(f.getPose());
                s.forDistance(dist);
            } else {
                s.hoodfar();
                s.shootc=1000;
                dist = shootp.distanceFrom(f.getPose());
                s.forDistance(dist);
            }
            tu.face(getShootTarget(),f.getPose());
            tu.automatic();
    }
    public void setTurretOffset(){
        if(a)tu.tti=1.5707963268;
        else tu.tti=1.62;
    }
    public void setShootTarget() {
        if (a){
            shootp = new Pose(0, 144, 0);
            parkPose= new Pose(106,34,Math.toRadians(90));
        }
        else {
            shootp = new Pose(144, 144, 0);
            parkPose= new Pose(38,34,Math.toRadians(90));
        }
    }
    public Pose getShootTarget() {
        return shootp;
    }

}
