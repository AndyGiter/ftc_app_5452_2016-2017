package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/12/2017.
 */
@Autonomous(name = "Move Shoot Bump ONE BALL ALL SIDES", group ="auto")
public class MoveShootBumpOneBall extends LinearBase {

    public void runOpMode()
    {
        initialise(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        sleep(100);

        sleep(17500); // to give allies time to move

        moveShootMove(-0.3, 1440 * 2.85, 1440 * 1.3);
    }
}
