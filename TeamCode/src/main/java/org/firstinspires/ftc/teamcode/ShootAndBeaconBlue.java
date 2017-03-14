package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/11/2017.
 */

@Autonomous(name = "Shoot and Beacon, BLUE SIDE", group ="auto")
public class ShootAndBeaconBlue extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        Thread.sleep(100);
        frontColor.enableLed(true);

        moveShootMove(-0.5, 1440 * 1.15, 1440 * 1.15);

        turn(0.3, -45); // turn right 45

        move(-0.7, 1440 * 1.7); // the diag

        turn(0.45, 133); // turn to face the button

        pressAndTest(0.4, Color.BLUE);

        move(-0.5, 1440 * 2.5);
    }
}
