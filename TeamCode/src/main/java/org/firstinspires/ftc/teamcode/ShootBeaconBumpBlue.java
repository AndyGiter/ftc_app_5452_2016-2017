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

    public void moveShootMove(double speed, double totalDist, double distBeforeShoot) throws InterruptedException
    {
        if(distBeforeShoot != 0)
            move(speed, distBeforeShoot);

        cannon.setPosition(0);
        Thread.sleep(200);

        move(speed, totalDist - distBeforeShoot);
    }
}
