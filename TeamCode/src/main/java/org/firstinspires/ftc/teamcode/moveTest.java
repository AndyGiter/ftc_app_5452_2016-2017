package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 4/6/2017.
 */

@Autonomous(name = "Move Test", group="testing")
public class moveTest extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initialise(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        Thread.sleep(100);

        move(MAX_MOVE_SPEED, 1440);
    }
}
