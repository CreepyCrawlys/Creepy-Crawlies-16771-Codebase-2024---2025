 /*
 Copyright 2024 FIRST Tech Challenge Team FTC

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial
 portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
 package org.firstinspires.ftc.teamcode.DriverCode;

 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver Main (Use ME!!!)")
 public class Driver1 extends OpMode {
     /* Declare OpMode members. */
     private DcMotor armLifter1;
     private DcMotor armLifter2;
     private  DcMotor pullyMotor;
     private DcMotor backLeft;
     private DcMotor backRight;
     private DcMotor frontLeft;
     private DcMotor frontRight;
     private Servo grabberLeft;
     private Servo grabberRight;
     private Servo rotate;
     private ElapsedTime runtime;
     boolean isPressed;
     double frontLeftPower;
     double frontRightPower;
     double backLeftPower;
     double backRightPower;

     private double gp1_left_bump_last_press;
     private double gp2_left_bump_last_press;
     private double gp2_right_bump_last_press;

     private boolean canHang = false;
     boolean finaleMovement = false;


     @Override
     public void init() {
         telemetry.addData("Status", "Initialized");
         armLifter1 = hardwareMap.get(DcMotor.class, "armMotor1");
         armLifter2 = hardwareMap.get(DcMotor.class, "armMotor2");
         backLeft = hardwareMap.get(DcMotor.class, "leftBack");
         backRight = hardwareMap.get(DcMotor.class, "rightBack");
         frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
         frontRight = hardwareMap.get(DcMotor.class, "rightFront");
         grabberLeft = hardwareMap.get(Servo.class, "leftArmGraber");
         grabberRight = hardwareMap.get(Servo.class, "rightArmGraber");
         rotate = hardwareMap.get(Servo.class, "graberS");
         pullyMotor = hardwareMap.get(DcMotor.class, "pully");

         runtime = new ElapsedTime();

         gp1_left_bump_last_press = 0;

         gp2_left_bump_last_press = 0;
         gp2_right_bump_last_press = 0;
     }

     @Override
     public void start() {

         armLifter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         armLifter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     }




    @Override
     public void loop() {
        double leftY = gamepad1.left_stick_y * 2;
        double rightY = gamepad1.right_stick_y * 2;  // For controlling the right side
        double leftX = gamepad1.left_stick_x * 2;   // For controlling sideways movement
        double rightX = gamepad1.right_stick_x * -4;
        double rightTrigger = gamepad1.right_trigger;
        double leftTrigger = gamepad1.left_trigger;

        finaleMovement = false;

        armLift();//Tank Drive
        grabberControl();
        pullyControll();

            frontLeftPower = leftY + leftX;
            frontRightPower = rightY - rightX;
            backLeftPower = leftY - leftX;
            backRightPower = rightY + rightX;


            if (gamepad1.left_bumper) {
                double nowTime = runtime.seconds();
                if ((gp1_left_bump_last_press + 0.3) < nowTime) {
                    gp1_left_bump_last_press = nowTime;

                    finaleMovement = !finaleMovement;
                }
            }

            // Check if the right button is pressed and slow the robot down
            if (gamepad1.x) {
                frontLeftPower /= 4;
                backLeftPower /= 4;
                frontRightPower /= 4;
                backLeftPower /= 4;
            }

            // Set the power of the motors
            frontLeft.setPower(-frontRightPower / 2);
            frontRight.setPower(-frontLeftPower / 2);
            backLeft.setPower(-backRightPower / 2);
            backRight.setPower(backLeftPower / 2);



    }
     public void grabberControl(){


         if(gamepad2.left_bumper){
                 double nowTime = runtime.seconds();
                 if ((gp2_left_bump_last_press + 0.3) < nowTime) {
                     gp2_left_bump_last_press = nowTime;
                     if (grabberRight.getPosition()==0){
                         grabberRight.setPosition(1);  //open
                         grabberLeft.setPosition(1);
                     }else {
                         grabberRight.setPosition(0);    //close
                         grabberLeft.setPosition(0);
                     }
                 }
             }


         if(gamepad2.right_bumper){
                 double nowTime = runtime.seconds();
                 if ((gp2_right_bump_last_press + 0.3) < nowTime) {
                     gp2_right_bump_last_press = nowTime;
                     if (rotate.getPosition()==0){
                         rotate.setPosition(1);
                     }else {
                         rotate.setPosition(0);
                     }
                 }
             }
     }

     double lastPos = 0;
     public void armLift(){
         double currentPos = gamepad2.left_stick_y/2;
         if(-0.1 <= currentPos && currentPos <= 0.1){
             armLifter1.setPower(0);
             armLifter2.setPower(0);
         }
         armLifter1.setPower(currentPos);
         armLifter2.setPower(-currentPos);

         lastPos = currentPos;
     }

     public void pullyControll(){
         if(gamepad1.dpad_down && gamepad2.dpad_down){
             canHang = true;
         }
         if(canHang){
             if(gamepad2.triangle) pullyMotor.setPower(1);
             if(gamepad2.x) pullyMotor.setPower(-1);
         }
     }

}
