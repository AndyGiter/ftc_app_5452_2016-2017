package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 4/24/2017.
 */

@Autonomous(name = "Right Ramp, ONE BALLS", group ="auto")
public class RightRampOneBall extends LinearBase{

    public void runOpMode()
    {
        initialise(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        sleep(100);

        moveShootMove(MAX_MOVE_SPEED * -1, 1440 * 1.8, 1440 * 1.3);

        turn(-60);

        sleep(13000); // Change to give teams time to move, test the max value.

        move(-1440*3);
    }
}
