package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by mlowery2 on 12/3/2016.
 */
@Autonomous(name = "Shoot and Bump, STRAIGHT", group ="auto")
public class ShootAndBumpStraight extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize();
        waitForStart();
        Thread.sleep(100);

        shoot();

        move(0.75, 1440*7); // Untested
    }

    public void shoot() throws InterruptedException
    {
        cannon.setPosition(0);
        Thread.sleep(200);
    }
}
