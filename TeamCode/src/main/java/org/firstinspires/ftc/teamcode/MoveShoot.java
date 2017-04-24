package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/12/2017.
 */

@Autonomous(name = "Move Shoot ALL SIDES", group ="auto")
public class MoveShoot extends LinearBase {

    public void runOpMode()
    {
        initialise(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        sleep(100);

        sleep(10000);

        moveShootMove(-0.3, 1440 * 1.8, 1440 * 1.8);

        turn(180);
    }
}
