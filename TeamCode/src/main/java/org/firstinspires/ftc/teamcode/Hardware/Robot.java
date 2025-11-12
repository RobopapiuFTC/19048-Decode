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
    private Shooter s;
    private Intake i;
    private Movement m;
    private Camera c;
    public static Pose startingPose = new Pose(72,135,0);
    public static Pose shootp = new Pose(5,130,0);
    public static double red,blue,green;
    public Gamepad g1,g2;
    public Follower f;
    private boolean a,shoot,oks;
    private boolean ro = true;
    public double speed = 0.9;
    public Timer iTimer,rTimer,rsTimer,sTimer;
    public boolean da=true,r,y,b,need=false,state = false,ts=false,spec=false,daS=false;
    public int flip = 1, iState = -1;
    public static int offset=20;

    public Robot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, boolean blue,boolean spec) {
        this.h = h;
        this.t = t;
        this.g1 = g1;
        this.g2 = g2;
        this.a = blue;
        this.spec = spec;
        f = Constants.createFollower(this.h);
        f.setStartingPose(startingPose);
        f.update();
        s=new Shooter(this.h,this.t);
        m=new Movement(this.h, this.t);
        i=new Intake(this.h,this.t);
       // c=new Camera(this.h,this.t,blue);

        iTimer = new Timer();
        rTimer = new Timer();
        rsTimer = new Timer();
        sTimer = new Timer();
    }

    public void tPeriodic() {
        //t.addData("Velocity: ", s.SD.getVelocity());
        t.addData("Turret Ticks", s.turret.getCurrentPosition());
        t.addData("Target", s.targett);
        t.addData("Power", s.power);
        sequenceshoot();
      //  c.periodic();
        turret();
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
    public void dualControls(){
        if(g1.dpad_up)s.target=2000;
        if(g1.dpad_right)s.target=1500;
        if(g1.dpad_down)s.target=-500;
        if(g1.dpad_left)s.turret.setPower(1);
        if(g1.b)s.hoodfar();
        if(g1.y){
            if(iTimer.getElapsedTimeSeconds()>0.3){
                sequenceintake();
                iTimer.resetTimer();
            }
        }
        if(g1.x){
            if(iTimer.getElapsedTimeSeconds()>0.3){
                shoot=true;
                oks=true;
                iTimer.resetTimer();
            }
        }
        if(g1.a)i.pornit=true;
    }
    public void sequenceintake(){
        s.target=-200;
        i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        i.pornit=true;
    }
    public void sequenceshoot(){
        if(shoot) {
            if(oks){
                sTimer.resetTimer();
                oks=false;
            }
            if(sTimer.getElapsedTimeSeconds()<0.2){
                i.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                i.pornit=true;
                s.target=-200;
            }
            if(sTimer.getElapsedTimeSeconds()>0.2 && sTimer.getElapsedTimeSeconds()<1){
                i.pornit=false;
                i.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                s.target=1500;
                shoot=false;
                oks=false;
            }
        }
    }
    public void turret() {
       /* double cameramaxy = 26;
        double turretmaxdegree = 26;
        double ticksfor360 = 1900;
        double maxticks = 1900;
        double ticksperdegree = ticksfor360 / 360;
        double tickspercameradegree = ticksperdegree / (cameramaxy / turretmaxdegree);
        double ticksneeded = -c.tx * tickspercameradegree;
        if (s.turret.getCurrentPosition() + ticksneeded > maxticks) {
            sTimer.resetTimer();
            s.targett = s.turret.getCurrentPosition() + ticksneeded - maxticks;
        }
        else if (s.turret.getCurrentPosition()+ticksneeded<0){
            sTimer.resetTimer();
            s.targett=s.turret.getCurrentPosition()+ticksneeded+maxticks;
        }
        else s.targett=s.turret.getCurrentPosition()+ticksneeded; */
        f.update();
        double c1 = f.getPose().getX()-shootp.getX();
        double c2 = shootp.getY()-f.getPose().getY();
        double ipoten = Math.sqrt(c1*c1+c2*c2);
        double unghi = Math.toDegrees(asin(c1/ipoten));
        double degreestointake=90;
        double ticksfor360 = 1900;
        double maxticks = 1900;
        double ticksperdegree = ticksfor360 / 360;
        double ticksneeded=(unghi+degreestointake-Math.toDegrees(f.getPose().getHeading()))*ticksperdegree;
        s.targett=ticksneeded;

    }
}
