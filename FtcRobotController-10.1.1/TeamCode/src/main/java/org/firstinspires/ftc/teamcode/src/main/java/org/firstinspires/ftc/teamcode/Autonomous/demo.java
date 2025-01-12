package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class demo extends PID_Core {
    public DcMotor fR;  // Front Right motor
    public DcMotor bR;  // Back Right motor
    public DcMotor fL;  // Front Left motor
    public DcMotor bL;  // Back Left motor




    String rightFront = "rightFront";  // Get Front motor by name
    String rightBack = "rightBack";  // Back Right motor
    String leftFront = "leftFront";  // Front Right motor"leftFront
    String leftBack = "leftBack";// Front Left motor



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Test", "Test");
        PID_Core_Fetch(hardwareMap, rightFront, rightBack, leftFront, leftBack);

        waitForStart();
        telemetry.addData("Test", "Test2");

        driveStraight(0.3, 90, 1);
       //turnToAngle(90, 0.4);


        super.runOpMode();
    }

    @Override
    public void PID_Core_Fetch(HardwareMap hardwareMap, String frontLeftName, String frontRightName, String backLeftName, String backRightName) {
        super.PID_Core_Fetch(hardwareMap, frontLeftName, frontRightName, backLeftName, backRightName);
    }

}