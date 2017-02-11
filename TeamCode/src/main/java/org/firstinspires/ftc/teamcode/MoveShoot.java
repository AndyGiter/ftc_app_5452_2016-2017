package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by mlowery2 on 1/12/2017.
 */

@Autonomous(name = "Move Shoot ALL SIDES", group ="auto")
public class MoveShoot extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize();
        waitForStart();
        Thread.sleep(100);

        moveShootMove(-0.75, 1440 * 3.5, 1440*3.5);

        Thread.sleep(10000);

        turn(1, 180);
    }
}
