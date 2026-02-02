package org.firstinspires.ftc.teamcode.Hardware;
import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.asin;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class Robot {
    private HardwareMap h;
    private TelemetryManager t;
    public Shooter s;
    public Intake i;
    private Movement m;
    public Turret tu;
    public double dist;
    public static Pose shootp = new Pose(0 ,144,0);
    public static Pose parkPose,endPose,startingPose,currentPose,futurePose;
    public Gamepad g1,g2;
    public Follower f;
    public static boolean a,shoot,oks,aim,auto,intake,oki,pids=false,aima=true,shooting=false,aiming=true;
    public Timer iTimer,rTimer,rsTimer,sTimer,oTimer;
    public Timer looptimer;
    public int loops;
    public double loopTime,lastLoop,timing=0.2;
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
        this.startingPose=startingPose;
        s=new Shooter(this.h,this.t);
        m=new Movement(this.h, this.t);
        i=new Intake(this.h,this.t);
        tu=new Turret(this.h,this.t,this.a);

        iTimer = new Timer();
        rTimer = new Timer();
        rsTimer = new Timer();
        sTimer = new Timer();
        oTimer = new Timer();
        looptimer = new Timer();
        looptimer.resetTimer();
    }
    public void stop() {
        endPose = f.getPose();
    }

    public void tPeriodic() {
        poses();
        setShootTarget();
        sequenceshoot();
        sequenceintake();
        isFull();
        hood();
        if(shooting){
            shooting();
            sc();
        }
        if(!tu.manual) {
            //if (aim) turret();
            if(aim)sotm();
            else {
                tu.setYaw(0);
            }
        }
        else tu.setYaw(Math.toRadians(90));
        i.periodic();
        s.periodic();
        tu.periodic();
        loops++;

        if (loops > 10) {
            double now = looptimer.getElapsedTime();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }
    }
    public void aPeriodic(){
        poses();
        sequenceshoot();
        hood();
        sequenceintake();
        //turret();
        sotm();
        if(shooting)shooting();
        if(!aiming)tu.setYaw(0);
        if(pids){
            s.periodic();
            tu.periodic();
        }
        i.periodic();
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
        setTurretOffset();
        tu.automatic();
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
        if(g1.dpad_left && !g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                tu.offset=tu.offset+0.0872665;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_right && !g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                tu.offset=tu.offset-0.0872665;
                oTimer.resetTimer();
            }
        }if(g1.dpad_left && g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                tu.offset=tu.offset+1.5707963268;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_right && g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                tu.offset=tu.offset-1.5707963268;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_up){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                s.offset=s.offset+10;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_down){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                s.offset=s.offset-10;
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
        slowmode = g1.right_trigger > 0.3;
        if(g1.left_trigger > 0.3 && g1.right_bumper)f.setPose(startingPose);
        if(g1.options){
            if(rTimer.getElapsedTimeSeconds()>0.3){
                tu.manual=!tu.manual;
                rTimer.resetTimer();
            }
        }
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
    public void poses(){
        currentPose=f.getPose();
        futurePose = new Pose(currentPose.getX()+f.getVelocity().getXComponent()*f.getAcceleration().getXComponent()*0.2, currentPose.getY()+f.getVelocity().getYComponent()*f.getAcceleration().getYComponent()*0.2,currentPose.getHeading());
    }
    public void sotm() {
            if(a){
                if(futurePose.getX()<80){
                    if (futurePose.getY() > 40) {
                        shootp = new Pose(0, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(4, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    }
                }
                else{
                    if (futurePose.getY() > 40) {
                        shootp = new Pose(0, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(4, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    }
                }
            }
            else{
                if(futurePose.getX()<80) {
                    if (futurePose.getY() > 40) {
                        shootp = new Pose(144, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(140, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    }
                }
                else{
                    if (futurePose.getY() > 40) {
                        shootp = new Pose(144, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    } else {
                        shootp = new Pose(140, 144, 0);
                        dist = shootp.distanceFrom(futurePose);
                        s.forDistance(dist);
                    }
                }
            }
            if(aiming){
                tu.face(getShootTarget(),futurePose);
                tu.automatic();
            }
    }
    public void turret() {
        if(a){
            if(currentPose.getX()<80){
                if (currentPose.getY() > 40) {
                    shootp = new Pose(0, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                } else {
                    shootp = new Pose(4, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                }
            }
            else{
                if (currentPose.getY() > 40) {
                    shootp = new Pose(0, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                } else {
                    shootp = new Pose(4, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                }
            }
        }
        else{
            if(currentPose.getX()<80) {
                if (currentPose.getY() > 40) {
                    shootp = new Pose(144, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                } else {
                    shootp = new Pose(140, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                }
            }
            else{
                if (currentPose.getY() > 40) {
                    shootp = new Pose(144, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                } else {
                    shootp = new Pose(140, 144, 0);
                    dist = shootp.distanceFrom(currentPose);
                    s.forDistance(dist);
                }
            }
        }
        if(aiming){
            tu.face(getShootTarget(),currentPose);
            tu.automatic();
        }
    }
    public void setTurretOffset(){
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
            if (currentPose.getX() < 72) s.shootc = s.shootn;
            else s.shootc = s.shootn - 30;
        }
        else{
            if (currentPose.getX() > 72) s.shootc = s.shootn;
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
    public double getLoopTimeMs(){
        return loopTime;
    }
    public double getLoopTimeHz(){
        return 1000/loopTime;
    }

    public void isFull(){
        if(!i.pornit)return;
        if(i.getVelocity()>-900)g1.rumble(200);
    }



}
