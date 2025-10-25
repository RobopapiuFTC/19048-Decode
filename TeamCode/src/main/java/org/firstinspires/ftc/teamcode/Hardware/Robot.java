package org.firstinspires.ftc.teamcode.Hardware;
import com.pedropathing.follower.Follower;
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

public class Robot {
    private HardwareMap h;
    private Telemetry t;
    private Shooter s;
    private Intake i;
    private Movement m;
    public static double red,blue,green;
    public Gamepad g1,g2;
    public Follower f;
    private boolean a;
    private boolean ro = true;
    public double speed = 0.9;
    public Timer iTimer,rTimer,rsTimer,sTimer;
    public boolean da=true,r,y,b,need=false,running = false,ts=false,spec=false,daS=false;
    public int flip = 1, iState = -1;
    public static int offset=20;

    public Robot(HardwareMap h, Telemetry t, Gamepad g1, Gamepad g2, boolean blue,boolean spec) {
        this.h = h;
        this.t = t;
        this.g1 = g1;
        this.g2 = g2;
        this.a = blue;
        this.spec = spec;

        s=new Shooter(this.h,this.t);
        m=new Movement(this.h, this.t);
        i=new Intake(this.h,this.t);

        iTimer = new Timer();
        rTimer = new Timer();
        rsTimer = new Timer();
        sTimer = new Timer();
    }

    public void tPeriodic() {
        m.periodic(g1);
        i.periodic();
        s.periodic();
        t.update();
    }
    public void tInit(){

    }
    public void dualControls(){
        if(g1.dpad_up)s.target=100;
        if(g1.dpad_right)s.target=50;
        if(g1.dpad_down)s.target=0;
        if(g1.b)s.hoodfar();
        if(g1.y){
            if(iTimer.getElapsedTimeSeconds()>0.3){
                i.pornit=!i.pornit;
                iTimer.resetTimer();
            }
        }

    }
}
