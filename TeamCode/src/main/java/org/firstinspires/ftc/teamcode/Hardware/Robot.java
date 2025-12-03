package org.firstinspires.ftc.teamcode.Hardware;
import static java.lang.Math.asin;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

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

public class Robot {
    private HardwareMap h;
    private Telemetry t;
    public Shooter s;
    public Intake i;
    private Movement m;
    private Camera c;
    public static Pose startingPose = new Pose(72,135,Math.toRadians(90));
    public static Pose shootp = new Pose(-5 ,144,0);
    public static double red,blue,green;
    public Gamepad g1,g2;
    public Follower f;
    public boolean a,shoot,oks,aim,auto,intake,oki,pids=false;
    public double c1,c2,ipoten,unghi,ticksneeded;
    private boolean ro = true;
    public double speed = 0.9;
    public Timer iTimer,rTimer,rsTimer,sTimer,oTimer;
    public boolean da=true,r,y,b,need=false,state = false,ts=false,spec=false,daS=false;
    public int flip = 1, iState = -1;
    public static int offset=0;
    double ticksfor360 = 1900;
    double maxticks = 1900;
    double ticksperdegree = ticksfor360 / 360;
    public Robot(HardwareMap h,Follower f, Telemetry t, Gamepad g1, Gamepad g2, boolean blue,boolean spec,boolean auto) {
        this.h = h;
        this.t = t;
        this.f = f;
        this.g1 = g1;
        this.g2 = g2;
        this.a = blue;
        this.spec = spec;
        this.auto = auto;
        s=new Shooter(this.h,this.t);
        m=new Movement(this.h, this.t);
        i=new Intake(this.h,this.t);
       // c=new Camera(this.h,this.t,blue);

        iTimer = new Timer();
        rTimer = new Timer();
        rsTimer = new Timer();
        sTimer = new Timer();
        oTimer = new Timer();
    }

    public void tPeriodic() {
        t.addData("Velocity: ", s.SD.getVelocity());
        t.addData("Turret Ticks", s.turret.getCurrentPosition());
        t.addData("Target", s.targett);
        t.addData("Power", s.power);
        sequenceshoot();
        sequenceintake();
      //  c.periodic();
        //set starting pose aici pentru follower in teleop numa
        f.update();
        if(aim)turret();
        t.addData("Pose x", f.getPose().getX());
        t.addData("Pose y", f.getPose().getY());
        t.addData("Pose heading", Math.toDegrees(f.getPose().getHeading()));
        m.periodic(g1);
        i.periodic();
        s.periodic();
        t.update();
    }
    public void tInit(){
       // c.start();
    }
    public void aPeriodic(){
        sequenceshoot();
        sequenceintake();
        if(aim)turret();
        else s.targett=390;
        if(pids)s.periodic();
        i.periodic();
        t.update();
    }
    public void dualControls(){
        if(g1.b)s.hoodfar();
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
                offset+=5;
                oTimer.resetTimer();
            }
        }
        if(g1.dpad_right){
            if(oTimer.getElapsedTimeSeconds()>0.5){
                offset-=5;
                oTimer.resetTimer();
            }
        }
        if(g1.a)i.pornit=true;
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
    public void calculatetarget(){
        c1 = f.getPose().getX()-shootp.getX();
        c2 = shootp.getY()-f.getPose().getY();
        ipoten = Math.sqrt(c1*c1+c2*c2);
        unghi = Math.toDegrees(asin(c1/ipoten));
        ticksneeded=(unghi-Math.toDegrees(f.getPose().getHeading())+180+offset)*ticksperdegree;
        if(ticksneeded>1900)ticksneeded=ticksneeded-1900;
    }
    public void turret() {
            if(!auto)calculatetarget();
            if (f.getPose().getY() > 40) {
                s.hoodclose();
                s.target = 1250;
            } else {
                s.hoodfar();
                s.target = 1500;
            }
        s.targett=ticksneeded;

    }
}
