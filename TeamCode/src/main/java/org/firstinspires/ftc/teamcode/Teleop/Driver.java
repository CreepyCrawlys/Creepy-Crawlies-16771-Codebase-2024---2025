package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Driver extends LinearOpMode {

    public final float rbSpeed = 0.5f;  // Speed constant for the robot
    public DcMotor fR;  // Front Right motor
    public DcMotor bR;  // Back Right motor
    public DcMotor fL;  // Front Left motor
    public DcMotor bL;  // Back Left motor

    public void initHardware() {
        // Accessing motors through hardwareMap (not HardwareMap.get())
        fR = hardwareMap.get(DcMotor.class, "Front");  // Get Front motor by name
        bR = hardwareMap.get(DcMotor.class, "BackRight");  // Back Right motor
        fL = hardwareMap.get(DcMotor.class, "FrontLeft");  // Front Left motor
        bL = hardwareMap.get(DcMotor.class, "BackLeft");  // Back Left motor

        // Setting motor directions if needed (depends on wiring)
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initHardware();

        // Wait for the start signal
        waitForStart();

        // Main control loop (this will run when the op mode is started)
        while (opModeIsActive()) {
            // Insert your control code here to drive the robot
            // For example, using gamepad inputs to control motors

            // Set power to motors
            fR.setPower(gamepad1.right_stick_y);  // Example: using right stick Y-axis for forward/reverse
            bR.setPower(gamepad1.right_stick_y);  // Example: using right stick Y-axis for forward/reverse
            fL.setPower(gamepad1.left_stick_y);   // Example: using left stick Y-axis for forward/reverse
            bL.setPower(gamepad1.left_stick_y);   // Example: using left stick Y-axis for forward/reverse

            // Optional: Add telemetry to show motor power values
            telemetry.addData("Front Right Power", fR.getPower());
            telemetry.addData("Back Right Power", bR.getPower());
            telemetry.addData("Front Left Power", fL.getPower());
            telemetry.addData("Back Left Power", bL.getPower());
            telemetry.update();

            // Add any other functionality here (e.g., additional controls)
        }
    }
}
