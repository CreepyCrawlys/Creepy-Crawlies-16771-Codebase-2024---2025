package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class AutoBase extends LinearOpMode {

    // Drive Motors (Front Right, Back Right, Front Left, Back Left motors)
    protected DcMotor fR, bR, fL, bL;

    // Sensors and odometry
    protected IMU imu;
    protected YawPitchRollAngles robotOrientation;
    private double currentYaw;
    private boolean isDriving = true;

    // Initialize motors and sensors
    protected void initMotorsAndSensors(HardwareMap hardwareMap) {
        // Map motors
        fR = hardwareMap.get(DcMotor.class, "rightFront");  // Get Front motor by name
        bR = hardwareMap.get(DcMotor.class, "rightBack");  // Back Right motor
        fL = hardwareMap.get(DcMotor.class, "leftFront");  // Front Left motor
        bL = hardwareMap.get(DcMotor.class, "leftBack");

        // Set all motors' zero power behavior to brake
        setZeroPowerBehavior();

        // Initialize IMU for orientation tracking
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    // Set all motors' zero power behavior to brake
    private void setZeroPowerBehavior() {
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors and sensors
        initMotorsAndSensors(hardwareMap);

        // Wait for the game to start
        waitForStart();

        // Call the user's autonomous code
        runAuto();
    }

    // Abstract method to be implemented by users to define their autonomous path
    public abstract void runAuto();

    // Drive the robot straight
    protected void driveStraight(double power, int targetAngle, double targetDistance) {
        resetMotorEncoders();
        while (opModeIsActive() && isDriving) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Steering logic to keep robot driving straight
            double steeringAdjustment = (currentYaw - targetAngle) * 0.02;
            applySteering(steeringAdjustment, power);

            // Check if the robot has driven the desired distance
            if (getAverageDistance() >= targetDistance) {
                isDriving = false;
                sleep(500);
                stopMotors();
            }
        }
        isDriving = true;
    }

    // Reset the motor positions for deadwheel tracking
    private void resetMotorEncoders() {
        fL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Calculate the average distance from all motors
    private double getAverageDistance() {
        return (fL.getCurrentPosition() + fR.getCurrentPosition() + bL.getCurrentPosition() + bR.getCurrentPosition()) / 4.0;
    }

    // Apply steering correction to drive motors
    private void applySteering(double steeringAdjustment, double power) {
        bL.setPower(-power - steeringAdjustment);
        bR.setPower(-power + steeringAdjustment);
        fL.setPower(power + steeringAdjustment);
        fR.setPower(-power + steeringAdjustment);
    }

    // Turn the robot to a target angle
    protected void turnToAngle(int targetAngle, double power) {
        currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        boolean turnRight = targetAngle > currentYaw;

        while (opModeIsActive() && isDriving) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Current Angle", currentYaw + " Target: " + targetAngle);
            telemetry.update();

            if (turnRight) {
                bL.setPower(power);
                bR.setPower(-power);
                fL.setPower(-power);
                fR.setPower(-power);
            } else {
                bL.setPower(-power);
                bR.setPower(power);
                fL.setPower(power);
                fR.setPower(power);
            }

            // Stop turning when the target angle is reached
            if ((turnRight && currentYaw >= targetAngle) || (!turnRight && currentYaw <= targetAngle)) {
                isDriving = false;
                sleep(500);
                stopMotors();
            }
        }
        isDriving = true;
    }

    // Stop all motors
    private void stopMotors() {
        fL.setPower(0);
        fR.setPower(0);
        bL.setPower(0);
        bR.setPower(0);
    }
}
