//package org.firstinspires.ftc.teamcode.Teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//
////Bitte gebrauch git, ich weis dass du es nich lernen wirst nico....
//
//@TeleOp
//public class TeleopTest extends OpMode {
//
//    String initMessage = "The code has been  not initzed";
//    Servo armServo1;
//    Servo armServo2;
//    Servo armServo3;
//
//
//    @Override
//    public void init() {
//        telemetry.addData("Title", initMessage); //Dies ist nur ein test code, zu sehen ob es funkt
//        armServo1 = hardwareMap.get(Servo.class, "armServo1");
//        armServo2 = hardwareMap.get(Servo.class, "armServo2");
//        armServo3 = hardwareMap.get(Servo.class, "armServo3");
//    }
//
//    @Override
//    public void loop() {
//        if(gamepad1.x){
//            armServo1.setPosition(0.5);
//        }
//
//        if(gamepad1.y){
//            armServo2.setPosition(0.5);
//        }
//
//        if(gamepad1.a){
//            armServo3.setPosition(0.5);
//        }
//
//    }
//}
