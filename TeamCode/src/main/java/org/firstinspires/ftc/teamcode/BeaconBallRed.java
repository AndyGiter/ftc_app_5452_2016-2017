package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/11/2017.
 */

@Autonomous(name = "One Beacon & Ball, RED SIDE", group ="auto")
public class BeaconBallRed extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initAndWait(DcMotor.RunMode.RUN_TO_POSITION, true);

        moveShootMove(-0.5, 1440*1.15, 1440*1.15);

        turn(0.3, 45); // turn left 45

        move(-0.7, 1440 * 1.3); // the diag

        turn(0.45, -133); // turn to face the button

        pressAndTest(0.4, Color.RED);

        //move(-0.5, 1440*2);

        //turn(0.5, -180);

        //move(0.5, 1440 * 0.5);
    }
}

