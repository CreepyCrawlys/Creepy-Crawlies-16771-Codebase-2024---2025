package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@Autonomous
public class AutoWithSide extends LinearOpMode {
    private DcMotor backLeft, backRight, frontLeft, frontRight, armLifter1, armLifter2;
    private Servo grabberLeft, grabberRight, rotate;

    private Rev2mDistanceSensor frontLeftDis, frontRightDis;

    IMU imu;

    double Yaw;

    YawPitchRollAngles robotCurrentOrientation;
    boolean loop = true;
    int M1C, M2C, M3C, M4C;
    Double tempLength;

    boolean coneDetected = false;
    String side = null;


    @Override
    public void runOpMode() throws InterruptedException {

        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        armLifter1 = hardwareMap.get(DcMotor.class, "armMotor1");
        armLifter2 = hardwareMap.get(DcMotor.class, "armMotor2");
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberLeft = hardwareMap.get(Servo.class, "leftArmGraber");
        grabberRight = hardwareMap.get(Servo.class, "rightArmGraber");
        rotate = hardwareMap.get(Servo.class, "graberS");


        imu = hardwareMap.get(IMU.class, "imu");


        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

          GrabberClose();
        sleep(1000);

        waitForStart();

        // Loop and update the dashboard
        imu.resetYaw();
        armLifter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLifter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLifter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLifter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //code for driving gos here --------------------------------------------------------------
        //TODO ACN YOU PLEAES NO USE NEGATIVE POWER
        GrabberClose();
        autoGrabberSet();

        driveSeide(0.3, 0, 0.67);//Go to the  speciment bar
        driveStraight(0.2, 0, -0.54);

        Armcontrol_mitscheiße(50, 0.3);//Put it on the specimen bar
        GrabberS(0.5);
        Armcontrol_mitscheiße(100, 1);
        sleep(800);
        GrabberOpen();
        Armcontrol_mitscheiße(0, 0.5);

        driveStraight(0.3, 0, 0.03);//Go to  the observation zones
        Armcontrol_mitscheiße(0, 0.4);
        driveStraight(0.3, 0, -0.01);
        driveSeide(0.3, 0, 0.6);
        driveStraight(0.3, 0, 0.01);


    }

    public void autoGrabberSet(){
        driveStraight(0.2, 0, -0.07);
        Armcontrol_mitscheiße(50, 0.2);
        GrabberS(0.1);
        Armcontrol_mitscheiße(0, 0.5);
        driveStraight(0.2, 0, 0.07);
        return;
    }


    public void driveStraight(double power, int angle, double meter) {

        double correctPower = 0.04;
        DistReset();
        while (opModeIsActive() && loop) {
            robotCurrentOrientation = imu.getRobotYawPitchRollAngles();
            Yaw = robotCurrentOrientation.getYaw(AngleUnit.DEGREES);



            if (meter > 0) {
                double x = (Yaw - angle) * -correctPower;
                DriveSteer(x, power);
                if (AverageDist() >= meter) {
                    loop = false;
                    sleep(500);
                    Stop(1, power);
                    sleep(10);
                }
            } else {
                double x = (Yaw - angle) * -correctPower;
                DriveSteer(x, -power);
                if (AverageDist() <= meter) {
                    loop = false;
                    sleep(500);
                    Stop(-1, power);
                    sleep(10);
                }
            }

        }
        loop = true;
    }

    public void DriveSteer(double turnFactor, double power) {
        backLeft.setPower(power - turnFactor);
        backRight.setPower(-power + turnFactor);
        frontLeft.setPower(power + turnFactor);//2,33
        frontRight.setPower(power + turnFactor);
    }

    public void DistReset() {
        M1C = backLeft.getCurrentPosition();
        M2C = backRight.getCurrentPosition();
        M3C = frontLeft.getCurrentPosition();
        M4C = frontRight.getCurrentPosition();
    }

    public double AverageDist() {
        double dist =((((double) (
                        (backRight.getCurrentPosition() - M2C) -
                        (frontLeft.getCurrentPosition() - M3C)
        )) / 2) / 4402);
        //4453
        telemetry.addData("left", frontLeft.getCurrentPosition());
        telemetry.addData("right", backRight.getCurrentPosition());
        telemetry.addData("Average", dist);
        telemetry.update();
        return dist;
    }

    public void driveSeide(double power, int angle, double meter) {

        DistReset();
        while (opModeIsActive() && loop) {

            robotCurrentOrientation = imu.getRobotYawPitchRollAngles();
            Yaw = robotCurrentOrientation.getYaw(AngleUnit.DEGREES);

            double x = (Yaw - angle) * -0.02;

            if (meter > 0) {
                DriveSteerSide(x, power);
                if (AverageDistSide() >= meter) {
                    loop = false;
                    sleep(500);
                    Stop(1, power);
                    sleep(10);
                }
            } else {
                DriveSteerSide(x, -power);
                if (AverageDistSide() <= meter) {
                    loop = false;
                    sleep(500);
                    Stop(1, power);
                    sleep(10);
                }
            }
            telemetry.addData("Average", AverageDistSide());
            telemetry.addData("meter", meter);
            telemetry.update();
        }
        loop = true;

    }

    public void DriveSteerSide(double turnFactor, double power) {
        backLeft.setPower(-power + turnFactor);
        backRight.setPower(-power + turnFactor);
        frontLeft.setPower(power + turnFactor);//2,33
        frontRight.setPower(-power - turnFactor);
    }

    public void SideStop(int fb, double power) {
        backLeft.setPower(power * 1.5 * fb);
        backRight.setPower(power * 1.5 * fb);
        frontLeft.setPower(-power * 1.5 * fb);
        frontRight.setPower(power * 1.5 * fb);
        sleep((long) power * 100);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }

    public double AverageDistSide() {
        return ((((double)
                (-(frontRight.getCurrentPosition() - M4C)
        )) / 4) / 4402);

    }

    public void Stop(int fb, double power) {
//        backLeft.setPower(-power * 1.5 * fb);
//        backRight.setPower(-power * 1.5 * fb);
//        frontLeft.setPower(power * 1.5 * fb);
//        frontRight.setPower(-power * 1.5 * fb);
//        sleep((long) power * 100);
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
    }


//    public void SetArmPos(int pos, double power) {
//        while (opModeIsActive() && loop) {
//            if (armLifter1.getCurrentPosition() <= -pos /32) {
//                telemetry.addData("dow\t", armLifter1.getCurrentPosition() + "\ntiks2\t" + armLifter2.getCurrentPosition());
//                telemetry.update();
//
//                armLifter1.setPower(power * -1);
//                armLifter2.setPower(-power * -1);
//
//                if (armLifter1.getCurrentPosition() >= -pos / 32) {
//                    loop = false;
//                    armLifter1.setPower(0);
//                    armLifter2.setPower(0);
//                }
//            } else if (armLifter1.getCurrentPosition() >= -pos / 32) {
//                telemetry.addData("up\t", armLifter1.getCurrentPosition() + "\ntiks2\t" + armLifter2.getCurrentPosition());
//                telemetry.update();
//
//                armLifter1.setPower(-power * -1);
//                armLifter2.setPower(power * -1);
//
//                if (armLifter1.getCurrentPosition() <= -pos / 32) {
//                    loop = false;
//                    armLifter1.setPower(0);
//                    armLifter2.setPower(0);
//                }
//            }
//        }
//        loop = true;
//   }

    //    public void GrabberControl(boolean open) {
//        if (open) {
//            grabberLeft.setPosition(0);
//            grabberRight.setPosition(0);
//        }
//        grabberLeft.setPosition(0.3);
//        grabberRight.setPosition(0.3);
//
//    }
    public void GrabberClose(){
        grabberLeft.setPosition(1);
        grabberRight.setPosition(1);
    }
    public void GrabberOpen(){
        grabberLeft.setPosition(0);
        grabberRight.setPosition(0);
    }

    public void GrabberS(double pos){
        rotate.setPosition(0.3);
    }
    public void turn(int angle, double power) {
        robotCurrentOrientation = imu.getRobotYawPitchRollAngles();
        Yaw = robotCurrentOrientation.getYaw(AngleUnit.DEGREES);
        boolean turnRechts = angle > Yaw;
        while (opModeIsActive() && loop) {
            robotCurrentOrientation = imu.getRobotYawPitchRollAngles();
            Yaw = robotCurrentOrientation.getYaw(AngleUnit.DEGREES);
            telemetry.addData("angle", Yaw + " " + angle);
            telemetry.update();
            if (turnRechts) {
                backLeft.setPower(power);
                backRight.setPower(-power);
                frontLeft.setPower(-power);
                frontRight.setPower(-power);
                sleep(10);
                if (angle < Yaw) {
                    loop = false;
                }
            } else {
                backLeft.setPower(-power);
                backRight.setPower(power);
                frontLeft.setPower(power);
                frontRight.setPower(power);
                sleep(10);
                if (angle > Yaw) {
                    loop = false;
                }
            }

        }
        loop = true;
        backLeft.setPower(-0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        //Stop(1,0.2);
        telemetry.addData("stoped", "");
        telemetry.update();

    }

    public void Armcontrol_mitscheiße( int position, double power) {
        int Grad_zahl = (700 / 180) * position;
        while (opModeIsActive() && loop) {
            if (armLifter1.getCurrentPosition() <= Grad_zahl) {
                telemetry.addData("down\t", armLifter1.getCurrentPosition() + "\nticks2\t" + armLifter2.getCurrentPosition());
                telemetry.update();

                armLifter1.setPower(power*0.3);
                armLifter2.setPower(-power*0.3);

                if (armLifter1.getCurrentPosition() >= Grad_zahl) {
                    loop = false;

                }
            } else if (armLifter1.getCurrentPosition() >= Grad_zahl) {
                telemetry.addData("up\t", armLifter1.getCurrentPosition() + "\nticks2\t" + armLifter2.getCurrentPosition());
                telemetry.update();

                armLifter1.setPower(-power*0.3);
                armLifter2.setPower(power*0.3);

                if (armLifter1.getCurrentPosition() <= Grad_zahl) {
                    loop = false;
                }
            }
        }

        double power_hold = 0.01;
        if (armLifter1.getCurrentPosition() < 350) {
            armLifter1.setPower(0.004);
            armLifter2.setPower(-0.004);

        } else {
            armLifter1.setPower(-0.004); // eigentich powerhold bei beiden muss dasselbe Vorzeichen haben
            armLifter2.setPower(0.004); // schaue aber im Test

        }
        loop = true;
    }
}