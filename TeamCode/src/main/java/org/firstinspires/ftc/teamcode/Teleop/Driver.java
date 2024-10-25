package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Driver extends LinearOpMode {

    public final float rbSpeed = 0.5f;
    public DcMotor fR;
    public DcMotor bR;
    public DcMotor fL;
    public DcMotor bL;


    public void initHardware(){
        fR = HardwareMap.get(DcMotor.class, "Front");


    }



    @Override
    public void runOpMode() throws InterruptedException {



    }
}
