package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by mlowery2 on 3/22/2017.
 */

@Autonomous(name = "Range Sensor Testing", group = "Testing")
public class SensorTester extends LinearBase{

    public void runOpMode() throws InterruptedException
    {
        initalize(true);
        waitForStart();
        Thread.sleep(100);

        while(opModeIsActive())
        {
            rangeTelemety(sonarRange);
            rangeTelemety(frontRange);
            telemetry.update();
        }
    }
}
