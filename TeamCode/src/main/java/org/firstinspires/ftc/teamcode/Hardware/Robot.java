package org.firstinspires.ftc.teamcode.Hardware;
import static androidx.core.math.MathUtils.clamp;
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
    public static Pose parkPose,endPose,startingPose=new Pose(72,135,Math.toRadians(90));
    public Gamepad g1,g2;
    public Follower f;
    public boolean a,shoot,oks,aim,auto,intake,oki,pids=false,aima=true,shooting=false,aiming=true;
    public Timer iTimer,rTimer,rsTimer,sTimer,oTimer;
    public boolean slowmode=false;
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
        hood();
        if(shooting){
            shooting();
            sc();
        }
        if(aim)turret();
        else{
            tu.setYaw(0);
        }
        //m.periodic(g1);
        i.periodic();
        s.periodic();
        tu.periodic(auto,aim);
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
        setTurretOffset();
    }
    public void aPeriodic(){
        sequenceshoot();
        hood();
        sequenceintake();
        turret();
        if(shooting)shooting();
        if(!aiming)tu.setYaw(0);
        if(pids){
            s.periodic();
            tu.periodic(auto,aim);
        }
        i.periodic();
    }
    public void aInit(){
        tu.resetTurret();
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
            i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            i.pornit=true;
            intake=false;
            oki=false;
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
        if(g1.a && !g1.left_bumper){
            shooting=false;
            i.pornit=true;
        }
        if(g1.a && g1.left_bumper){
            shooting=true;
        }
        if(g1.right_trigger>0.3)slowmode=true;
        else slowmode=false;
    }
    public void intake(){
        intake=true;
        oki=true;
        shooting=false;
    }
    public void shooter(){
        shoot=true;
        oks=true;
        aiming=true;
    }
    public void hood(){
        s.hood=clamp(0.05+(s.getTarget()-s.getVelocity()-20)*s.angle,0.05,0.5);
        s.SVD.setPosition(s.hood);
    }
    public void sequenceintake(){
        if(intake) {
            if (oki) {
                if(!auto){
                    s.off();
                }
                shooting=false;
                iTimer.resetTimer();
                oki = false;
            }
            if (iTimer.getElapsedTimeSeconds() < 0.6) {
                if(!auto)aim = false;
                s.latchup();
            }
            if (iTimer.getElapsedTimeSeconds() > 0.6 && iTimer.getElapsedTimeSeconds() < 1) {
                i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                i.pornit = true;
                intake=false;
                oki=false;
            }
        }
    }
    public void shooting(){
        if(shooting) {
            if (s.getVelocity() > s.getTarget() - 40) {
                i.pornit = true;
            }
            else i.pornit=false;
        }
    }
    public void sequenceshoot(){
        if(shoot) {
            if(oks){
                s.on();
                aim=true;
                sTimer.resetTimer();
                oks=false;
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
    public void shootauto(){
        if (f.getPose().getY() > 40) {
            s.shootc=970;
            dist = shootp.distanceFrom(f.getPose());
            s.forDistance(dist);
        } else {
            s.shootc=970;
            dist = shootp.distanceFrom(f.getPose());
            s.forDistance(dist);
        }
    }
    public void turretauto(){
        tu.face(getShootTarget(),f.getPose());
        tu.automatic();
    }
    public void turret() {
            if(a){
                if(f.getPose().getX()<80){
                    if (f.getPose().getY() > 40) {
                        shootp = new Pose(0, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(2, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    }
                }
                else{
                    if (f.getPose().getY() > 40) {
                        shootp = new Pose(0, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(2, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    }
                }
            }
            else{
                if(f.getPose().getX()<80) {
                    if (f.getPose().getY() > 40) {
                        shootp = new Pose(144, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(142, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    }
                }
                else{
                    if (f.getPose().getY() > 40) {
                        shootp = new Pose(144, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(142, 144, 0);
                        dist = shootp.distanceFrom(f.getPose());
                        s.forDistance(dist);
                    }
                }
            }
            if(aiming){
                tu.face(getShootTarget(),f.getPose());
                tu.automatic();
            }
    }
    public void setTurretOffset(){
        /*if(a)tu.tti=1.5707963268;
        else tu.tti=1.62; */
        tu.tti=1.5707963268;
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
    public void sc(){
        if(a) {
            if (f.getPose().getX() < 72) s.shootc = s.shootn;
            else s.shootc = s.shootn - 30;
        }
        else{
            if (f.getPose().getX() > 72) s.shootc = s.shootn;
            else s.shootc = s.shootn - 30;
        }
    }
    public void setShootTargetFar(){
        if (a){
            shootp = new Pose(2, 144, 0);
        }
        else {
            shootp = new Pose(142, 144, 0);
        }
    }
    public Pose getShootTarget() {
        return shootp;
    }

}
