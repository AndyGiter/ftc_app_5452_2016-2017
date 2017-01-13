package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
/**
 * Created by mlowery2 on 1/12/2017.
 */

@Autonomous(name = "Move Shoot Bump ALL SIDES", group ="auto")
public class MoveShootBump extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize();
        waitForStart();
        Thread.sleep(100);

        moveShootMove(-0.75, 1440 * 3, 1440 * 3.0);

        Thread.sleep(17500); // to give allys time to move

        move(-1 * MAX_MOVE_SPEED, 1440 * 3);
    }
}
