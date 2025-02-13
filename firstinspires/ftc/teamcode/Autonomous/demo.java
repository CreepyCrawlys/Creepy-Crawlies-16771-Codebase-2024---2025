package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MySimpleAuto")
public class demo extends OpMode {

    DcMotorEx arm1; //270
    DcMotorEx arm2; //270

    @Override
    public void init() {
        arm1 = hardwareMap.get(DcMotorEx.class, "armMotor1");
        arm2 = hardwareMap.get(DcMotorEx.class, "armMotor2");
    }

    @Override
    public void loop() {
        telemetry.addData("arm1 pos", arm1.getCurrentPosition());
        telemetry.addData("arm2 pos", arm2.getCurrentPosition());
        telemetry.update();
    }
}
