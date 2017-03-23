package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/12/2017.
 */

@Autonomous(name = "Move Shoot ALL SIDES", group ="auto")
public class MoveShoot extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        Thread.sleep(100);

        moveShootMove(-0.5, 1440 * 1.8, 1440 * 1.8);

        Thread.sleep(10000);

        turn(MAX_TURN_SPEED, 180);
    }
}
