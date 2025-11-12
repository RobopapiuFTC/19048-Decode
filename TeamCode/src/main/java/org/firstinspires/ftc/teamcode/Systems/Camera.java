package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Camera {
    private Limelight3A limelight;
    private Telemetry telemetry;
    private boolean blue;
    public double tx,ty,ta;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry, boolean blue){

        limelight=hardwareMap.get(Limelight3A.class, "limelight");
        if(blue)limelight.pipelineSwitch(3); //april tag pipeline blue
        else limelight.pipelineSwitch(4); //april tag pipeline red
        this.telemetry=telemetry;
    }

    public void periodic(){
        detect();
        telemetry.update();
    }
    public void start(){
        limelight.start();
    }
    public void detect(){
        LLResult result = limelight.getLatestResult();
        if(result!=null && result.isValid()){
            telemetry.addData("Target Position", result.getTx());
            telemetry.addData("Target Area", result.getTa());
            tx= result.getTx();
            ty= result.getTy();
            ta= result.getTa();
        }
    }
}