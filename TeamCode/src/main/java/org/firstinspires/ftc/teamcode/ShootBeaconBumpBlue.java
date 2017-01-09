package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by mlowery2 on 1/3/2017.
 */

@Autonomous(name = "Shoot Beacon Bump BLUE", group ="auto")
public class ShootBeaconBumpBlue extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize();
        waitForStart();
        Thread.sleep(100);

    }
}
