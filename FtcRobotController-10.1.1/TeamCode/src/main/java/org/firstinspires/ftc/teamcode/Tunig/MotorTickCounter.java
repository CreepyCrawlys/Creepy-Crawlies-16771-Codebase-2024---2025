package org.firstinspires.ftc.teamcode.Tunig;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp
public class MotorTickCounter extends OpMode {
    DcMotorEx baseMototor;
    double basemotorPos;
    @Override
    public void init() {
        baseMototor = hardwareMap.get(DcMotorEx.class, "rightFront");
    }

    @Override
    public void loop() {
        basemotorPos = baseMototor.getCurrentPosition();
        telemetry.addData("Base Motor Pos",basemotorPos);
    }
}
