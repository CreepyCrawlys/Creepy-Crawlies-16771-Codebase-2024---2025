package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Units;

public abstract class PID_Core extends LinearOpMode {

    // Drive Motors (updated config)
    private DcMotor fR, bR, fL, bL;  // Front Right, Back Right, Front Left, Back Left motors

    // Odometry (deadwheels)
    private DcMotor deadwheelLeft, deadwheelFront, deadwheelRight;  // Deadwheels for odometry

    // Sensors
    private Rev2mDistanceSensor frontLeftDistanceSensor, frontRightDistanceSensor;
    private IMU imu;
    private YawPitchRollAngles robotOrientation;

    // Odometry tracking variables
    private double currentYaw, robotPosX, robotPosY, robotOrientationTheta;
    private int lastBackLeftEncoder, lastBackRightEncoder, lastFrontLeftEncoder, lastFrontRightEncoder;
    private boolean isDriving = true;


    public void PID_Core_Fetch(HardwareMap hardwareMap, String frontLeftName, String frontRightName,
                               String backLeftName, String backRightName) {

        // Map motors
        fL = hardwareMap.get(DcMotor.class, frontLeftName);
        fR = hardwareMap.get(DcMotor.class, frontRightName);
        bL = hardwareMap.get(DcMotor.class, backLeftName);
        bR = hardwareMap.get(DcMotor.class, backRightName);

        // Initialize Deadwheel Motors
        deadwheelLeft = hardwareMap.get(DcMotor.class, backRightName);
        deadwheelFront = hardwareMap.get(DcMotor.class, frontRightName);
        deadwheelRight = hardwareMap.get(DcMotor.class, backLeftName);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        // Set zero power behavior for all motors
        setZeroPowerBehavior();

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        // Initialize variables
        robotOrientation = imu.getRobotYawPitchRollAngles();

        waitForStart();
    }

    // Set all motors' zero power behavior to brake
    private void setZeroPowerBehavior() {
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deadwheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deadwheelFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        deadwheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Drive the robot straight
    public void driveStraight(double power, int targetAngle, double targetDistance) {
        resetMotorEncoders();
        while (opModeIsActive() && isDriving) {
            telemetry.update();
            currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);

            // Steering logic to keep robot driving straight
            double steeringAdjustment = (currentYaw - targetAngle) * 0.02;
            applySteering(steeringAdjustment, power);

            // Check if the robot has driven the desired distance
            if (targetDistance > 0 && getAverageDistance() >= targetDistance) {
                isDriving = false;
                sleep(500);
                stopMotors();
            } else if (targetDistance <= 0 && getAverageDistance() <= targetDistance) {
                isDriving = false;
                sleep(500);
                stopMotors();
            }
        }
        isDriving = true;
    }

    // Calculate the average distance of all the wheels
    private double getAverageDistance() {
        return -((((double) (bL.getCurrentPosition() - lastBackLeftEncoder) +
                (bR.getCurrentPosition() - lastBackRightEncoder) +
                (fL.getCurrentPosition() - lastFrontLeftEncoder) +
                (fR.getCurrentPosition() - lastFrontRightEncoder)) / 4) / 1287);
    }

    // Reset the motor positions for deadwheel tracking
    private void resetMotorEncoders() {
        lastBackLeftEncoder = bL.getCurrentPosition();
        lastBackRightEncoder = bR.getCurrentPosition();
        lastFrontLeftEncoder = fL.getCurrentPosition();
        lastFrontRightEncoder = fR.getCurrentPosition();
    }
    // Apply steering correction to drive motors
    public void applySteering(double steeringAdjustment, double power) {
        bL.setPower(-power - steeringAdjustment);
        bR.setPower(-power + steeringAdjustment);
        fL.setPower(power + steeringAdjustment);
        fR.setPower(-power + steeringAdjustment);
    }

    // Stop all motors
    public void stopMotors() {
        bL.setPower(0);
        bR.setPower(0);
        fL.setPower(0);
        fR.setPower(0);
    }

    // Turn the robot to a target angle
    public void turnToAngle(int targetAngle, double power) {

        currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
        boolean turnRight = targetAngle > currentYaw;

        while (opModeIsActive() && isDriving) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            currentYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("Current Angle", currentYaw + " Target: " + targetAngle);
            telemetry.update();

            if (turnRight) {
                bL.setPower(power);
                bR.setPower(-power);
                fL.setPower(-power);
                fR.setPower(-power);
                sleep(10);
                if (currentYaw >= targetAngle) {
                    isDriving = false;
                }
            } else {
                bL.setPower(-power);
                bR.setPower(power);
                fL.setPower(power);
                fR.setPower(power);
                sleep(10);
                if (currentYaw <= targetAngle) {
                    isDriving = false;
                }
            }
        }
        isDriving = true;
        stopMotors();
    }
}