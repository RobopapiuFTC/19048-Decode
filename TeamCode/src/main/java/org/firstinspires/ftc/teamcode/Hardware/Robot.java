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
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    public static double dist,offsetFar=0,offsetClose=0,batteryVoltage=0,nominalVoltage=12.5;
    public static Pose shootp = new Pose(0 ,144,0);
    public static Pose parkPose,endPose,startingPose,currentPose,futurePose,relocalization;
    public Gamepad g1,g2;
    public Follower f;
    public VoltageSensor batteryVoltageSensor;
    public static boolean a,shoot,oks,aim,auto,intake,oki,pids=false,aima=true,shooting=false,aiming=true,rumble=false;
    public Timer iTimer,rTimer,rsTimer,sTimer,oTimer;
    public Timer looptimer;
    public int loops;
    public double loopTime,lastLoop,timing=0.7;
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
        offsets();
        setShootTarget();
        sequenceshoot();
        sequenceintake();
        isFull();
        rumble();
        setLatch();
        if(shooting){
            shooting();
            sc();
        }
        if(!tu.manual) {
           // if (aim) turret();
            if(aim)sotm();
            else {
                tu.setYaw(0);
            }
        }
        else tu.setYaw(Math.toRadians(90));
       // if(!s.activated)i.isFull();
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
        sequenceintake();
        turret();
        setLatch();
        //sotm();
        if(shooting)shooting();
        //if(!aiming)tu.setTurretTarget(0);
        if(pids){
            s.periodic();
            tu.periodic();
        }
        i.periodic();
    }
    public void aPeriodic2(){
        poses();
        shootTarget();
        if(aiming){
            tu.face(getShootTarget(),currentPose);
            tu.automatic();
        }
        s.periodic();
        tu.periodic();
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
    public void rumble(){
        if(!rumble)return;
        g1.rumble(1000);
        rumble=false;
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
            f.setPose(relocalization);
        }
        if(g1.dpad_left && !g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                if(currentPose.getY()>40) {
                    offsetClose = offsetClose + 0.0174533;
                    oTimer.resetTimer();
                }
                else {
                    offsetFar = offsetFar + 0.0174533;
                    oTimer.resetTimer();
                }
            }
        }
        if(g1.dpad_right && !g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                if(currentPose.getY()>40) {
                    offsetClose = offsetClose - 0.0174533;
                    oTimer.resetTimer();
                }
                else {
                    offsetFar = offsetFar - 0.0174533;
                    oTimer.resetTimer();
                }
            }
        }if(g1.dpad_left && g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                if(currentPose.getY()>40) {
                    offsetClose = offsetClose + 1.5707963268;
                    oTimer.resetTimer();
                }
                else {
                    offsetFar = offsetFar + 1.5707963268;
                    oTimer.resetTimer();
                }
            }
        }
        if(g1.dpad_right && g1.left_bumper){
            if(oTimer.getElapsedTimeSeconds()>0.3){
                if(currentPose.getY()>40) {
                    offsetClose = offsetClose - 1.5707963268;
                    oTimer.resetTimer();
                }
                else {
                    offsetFar = offsetFar - 1.5707963268;
                    oTimer.resetTimer();
                }
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
        if(g1.options){
            if(rTimer.getElapsedTimeSeconds()>0.3){
                tu.manual=!tu.manual;
                rTimer.resetTimer();
            }
        }
        if(g1.left_trigger > 0.3 && g1.right_bumper) {
            i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            i.pornit=true;
            intake=false;
            oki=false;
        }
    }
    public void offsets(){
        if(currentPose.getY()<40)tu.offset=offsetFar;
        else tu.offset=offsetClose;
    }
    public void setRelocalization(Pose relocalization){
        this.relocalization=relocalization;
    }
    public void intake(){
        intake=true;
        oki=true;
        shooting=false;
        i.third=false;
        i.second=false;
    }
    public void shooter(){
        shoot=true;
        oks=true;
        aiming=true;
        tu.okt=true;
    }
    public void sequenceintake(){
            if (intake) {
                if (oki) {
                    if (!auto) {
                        s.off();
                    }
                    shooting = false;
                    i.pornit = false;
                    iTimer.resetTimer();
                    oki = false;
                }
                if (iTimer.getElapsedTimeSeconds() < 0.6) {
                    if (!auto) aim = false;
                    s.latchup();
                }
                if (iTimer.getElapsedTimeSeconds() > 0.6 && iTimer.getElapsedTimeSeconds() < 1) {
                    i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    i.pornit = true;
                    intake = false;
                    oki = false;
                    i.oki = true;
                    i.looping = true;
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
                i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                s.latchdown();
                shoot=false;
                oks=false;
            }
        }
    }
    public void poses(){
        setShootTarget();
        currentPose=f.getPose();
        futurePose = new Pose(currentPose.getX()+f.getVelocity().getXComponent()*timing, currentPose.getY()+f.getVelocity().getYComponent()*timing,currentPose.getHeading());
    }
    public void sotm() {
           /* if(a){
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
            }*/
            if(futurePose.getY()>40)setShootTarget();
            else setShootTargetFar();
            dist = shootp.distanceFrom(futurePose);
            s.forDistance(dist);
            if(aiming){
                tu.face(getShootTarget(),futurePose);
                tu.automatic();
            }
    }
    public void turret() {
      /* if(a){
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
        }*/
        if(currentPose.getY()>40)setShootTarget();
        else setShootTargetFar();
        dist = shootp.distanceFrom(currentPose);
        s.forDistance(dist);
        if(aiming){
            tu.face(getShootTarget(),currentPose);
            tu.automatic();
        }
    }
    public void setLatch(){
        if(auto){
            s.latching=0.75;
        }
        else{
            s.latching=0.75;
        }
    }
    public void setTurretOffset(){
        tu.tti=-1.5707963268;
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
    public void shootTarget(){
        if(currentPose.getY()>40){
            setShootTarget();
        }
        else setShootTargetFar();
    }
    public void setShootTargetFar(){
        if (a){
            shootp = new Pose(2, 142, 0);
        }
        else {
            shootp = new Pose(142, 142, 0);
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
        if(i.getVelocity()<1300)g1.rumble(200);
    }



}
