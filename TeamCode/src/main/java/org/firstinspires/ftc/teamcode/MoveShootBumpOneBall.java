package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/12/2017.
 */
@Autonomous(name = "Move Shoot Bump ONE BALL ALL SIDES", group ="auto")
public class MoveShootBumpOneBall extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        Thread.sleep(100);

        moveShootMove(-0.75, 1440 * 1.6, 1440 * 1.6);

        Thread.sleep(17500); // to give allies time to move

        move(-0.5, 1440 * 1.25);
    }
}
