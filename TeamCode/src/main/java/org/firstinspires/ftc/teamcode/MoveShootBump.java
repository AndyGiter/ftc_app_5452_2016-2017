package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/12/2017.
 */

@Autonomous(name = "Move Shoot Bump ALL SIDES", group ="auto")
public class MoveShootBump extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initAndWait(DcMotor.RunMode.RUN_TO_POSITION, true);

        Thread.sleep(17500);

        moveShootMove(-0.5, 1440 * 1.8, 1440 * 2.85);
    }
}
