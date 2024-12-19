//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//@TeleOp
//public class Driver extends OpMode {
//
//    public final float rbSpeed = 0.5f;  // Speed constant for the robot
//    public DcMotor fR;  // Front Right motor
//    public DcMotor bR;  // Back Right motor
//    public DcMotor fL;  // Front Left motor
//    public DcMotor bL;  // Back Left motor
//    public DcMotor armMotor;
//
//    double motorSpeed;
//
//    public void initHardware() {
//        // Accessing motors through hardwareMap (not HardwareMap.get())
//        fR = hardwareMap.get(DcMotor.class, "frontRight");  // Get Front motor by name
//        bR = hardwareMap.get(DcMotor.class, "backRight");  // Back Right motor
//        fL = hardwareMap.get(DcMotor.class, "frontLeft");  // Front Left motor
//        bL = hardwareMap.get(DcMotor.class, "backLeft");  // Back Left motor
//        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
//
//        fR.setDirection(DcMotor.Direction.REVERSE);//The motor is inversed
//        bR.setDirection(DcMotor.Direction.REVERSE);
//        fL.setDirection(DcMotor.Direction.REVERSE);
//        bL.setDirection(DcMotor.Direction.FORWARD);
//    }
//
//
//    public void baseRobotController(){
//        fR.setPower(gamepad1.right_stick_y - gamepad1.right_stick_x);  // Example: using right stick Y-axis for forward/reverse
//        bR.setPower(gamepad1.right_stick_y + gamepad1.right_stick_x);  // Example: using right stick Y-axis for forward/reverse
//        fL.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);   // Example: using left stick Y-axis for forward/reverse
//        bL.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
//    }
//    private void armController(double armPower, double gearRatioScale) {
//        armPower = -gamepad2.left_stick_y;
//
//        armPower *= gearRatioScale;
//
//        armMotor.setPower(armPower);
//    }
//
//
//    @Override
//    public void init() {
//        initHardware();
//
//    }
//
//    @Override
//    public void loop() {
//        baseRobotController();
//        armController(gamepad2.right_stick_y, 0.651d);
//    }
//}
//
