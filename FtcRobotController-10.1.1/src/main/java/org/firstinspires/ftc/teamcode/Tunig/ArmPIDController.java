package org.firstinspires.ftc.teamcode.Tunig;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Deprecated
@Disabled
public class ArmPIDController {
    private final double kp, ki, kd;
    private double integralSum, lastError;
    private final ElapsedTime timer;

    public ArmPIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        integralSum = 0;
        lastError = 0;
        timer = new ElapsedTime();
    }

    public double calculateOutput(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        return (error * kp) + (integralSum * ki) + (derivative * kd);
    }
}
