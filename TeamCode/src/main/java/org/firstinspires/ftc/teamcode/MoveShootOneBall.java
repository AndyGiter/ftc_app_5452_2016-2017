package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/12/2017.
 */

@Autonomous(name = "Move Shoot ONE BALL ALL SIDES", group ="auto")
public class MoveShootOneBall extends LinearBase {

    public void runOpMode()
    {
        initialise(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        sleep(100);

        sleep(10000); // Make sure 10 seconds have passed

        moveShootMove(-0.3, 1440 * 1.3, 1440 * 1.3);

        turn(MAX_TURN_SPEED, 180);
    }
}
