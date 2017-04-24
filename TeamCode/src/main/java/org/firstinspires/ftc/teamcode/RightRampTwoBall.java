package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 4/24/2017.
 */

@Autonomous(name = "Right Ramp, TWO BALLS", group ="auto")
public class RightRampTwoBall extends LinearBase{

    public void runOpMode()
    {
        initialise(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        sleep(100);

        moveShootMove(MAX_MOVE_SPEED * -1, 1440 * 1.8, 1440 * 1.8);

        turn(-60);

        sleep(13000); // Change to give teams time to move, test the max value. Current estimate that everything up to this point takes 10 and the final move takes 5

        move(-1440*3);
    }
}
