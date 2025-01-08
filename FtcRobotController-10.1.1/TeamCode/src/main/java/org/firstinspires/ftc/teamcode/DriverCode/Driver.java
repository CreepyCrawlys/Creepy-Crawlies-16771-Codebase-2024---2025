package org.firstinspires.ftc.teamcode.DriverCode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tunig.armPIDConfig;

@TeleOp
public class Driver extends OpMode {

    public final float rbSpeed = 0.5f;  // Speed constant for the robot
    public DcMotor fR;  // Front Right motor
    public DcMotor bR;  // Back Right motor
    public DcMotor fL;  // Front Left motor
    public DcMotor bL;  // Back Left motor
    public DcMotorEx armMotor1;
    public DcMotorEx armMotor2;
    public PIDController armPID_Controller;
    public final double TICKSINDEGREES = 1344 / 180.0;


    double motorSpeed;

    public void initHardware() {
        // Accessing motors through hardwareMap (not HardwareMap.get())
        fR = hardwareMap.get(DcMotor.class, "rightFront");  // Get Front motor by name
        bR = hardwareMap.get(DcMotor.class, "rightBack");  // Back Right motor
        fL = hardwareMap.get(DcMotor.class, "leftFront");  // Front Left motor
        bL = hardwareMap.get(DcMotor.class, "leftBack");  // Back Left motor
        armMotor1 = hardwareMap.get(DcMotorEx.class, "armMotor1");
        armMotor2 = hardwareMap.get(DcMotorEx.class, "armMotor2");

        fR.setDirection(DcMotor.Direction.REVERSE);//The motor is inversed
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);

        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void initPID_Controllers(){

        armPID_Controller = new PIDController(armPIDConfig.P, armPIDConfig.I, armPIDConfig.D);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }


    public void baseRobotController(){
        fR.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x);  // Example: using right stick Y-axis for forward/reverse
        bR.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);  // Example: using right stick Y-axis for forward/reverse
        fL.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);   // Example: using left stick Y-axis for forward/reverse
        bL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);

    }
    private void armController(double targetPos) {

        if(gamepad2.right_stick_y < -0.5 ) armPIDConfig.TARGET -= 1;

        if(gamepad2.right_stick_y > 0.5 ) armPIDConfig.TARGET += 1;

        if(armPIDConfig.TARGET >= -47) armPIDConfig.TARGET = -47;
        if( armPIDConfig.TARGET <= -577) armPIDConfig.TARGET = -577; //-47 -577

        //++++++++++++++a+rmPIDConfig.target = (int) mapFloatToAngle(gamepad2.right_stick_y);

        armPID_Controller.setPID(armPIDConfig.P, armPIDConfig.I, armPIDConfig.D);

        int armPos = armMotor1.getCurrentPosition();
        int armPos2 = fL.getCurrentPosition();

        double pid = armPID_Controller.calculate(armPos2, armPIDConfig.TARGET);
//        double ff = Math.cos(Math.toRadians(armPIDConfig.target / TICKSINDEGREES)) * armPIDConfig.f;

        double power1 = pid;
        double power2 = pid;

        armMotor1.setPower(power1);
        armMotor2.setPower(power2);

        telemetry.addData("PID Value", pid);
        telemetry.addData("Pos2", armPos2);
        telemetry.addData("Target", armPIDConfig.TARGET);

        telemetry.update();
    }

    double mapFloatToAngle(float floatValue) {
        double angle = (floatValue + 1) * 180;
        angle = (angle + 180) % 360;
        return angle;
    }

    @Override
    public void init() {
        initHardware();
        initPID_Controllers();

    }

    @Override
    public void loop() {
        baseRobotController();
        armController(mapFloatToAngle(gamepad2.right_stick_y));
    }
}



