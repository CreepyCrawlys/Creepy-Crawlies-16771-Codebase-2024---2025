package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class PIDF_ArmControl extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0 , d = 0;
    public static double f = 0;

    public static int target = 0;

    private final double ticks_in_degree = 750 / 250.0;

    private DcMotorEx arm_motor1, arm_motor2;
    @Override
    public void init() {
        controller = new PIDController(p, i , d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor1 = hardwareMap.get(DcMotorEx.class, "armMotor1");
        arm_motor2 = hardwareMap.get(DcMotorEx.class, "armMotor2");

        arm_motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = arm_motor1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor1.setPower(power);
        arm_motor2.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}
