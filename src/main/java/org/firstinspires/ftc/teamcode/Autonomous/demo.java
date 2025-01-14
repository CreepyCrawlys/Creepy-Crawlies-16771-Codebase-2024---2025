package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="MySimpleAuto")
public class demo extends AutoBase {

    @Override
    public void runAuto() {
        // Example: Drive forward for 24 inches, then turn 90 degrees, then drive forward again
        driveStraight(0.2, 90, 24);  // Drive forward with 50% power for 24 inches
        sleep(1000);  // Wait for 1 second

        turnToAngle(90, 0.5);  // Turn 90 degrees to the right at 50% power
        sleep(1000);

        driveStraight(0.5, 90, 24);  // Drive forward with 50% power for another 24 inches
    }
}
