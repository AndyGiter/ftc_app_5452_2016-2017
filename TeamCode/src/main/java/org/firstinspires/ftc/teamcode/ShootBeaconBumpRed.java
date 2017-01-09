package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by mlowery2 on 1/3/2017.
 */

@Autonomous(name = "Shoot Beacon Bump RED", group ="auto")
public class ShootBeaconBumpRed extends LinearBase {

    public void runOpMode() throws InterruptedException {
        initalize();
        front.enableLed(false);
        waitForStart();
        Thread.sleep(100);

        pressAndTest(0.8, 1440, Color.RED); // apparently slower is better

        /*
        moveShootMove(0.75, 1440 * 1.4, 0); // forward 1440*1.4

        turn(0.4, -45); // right 45

        move(0.8, 1440); // forward 1440

        turn(0.4, -45); // right 45
        */
    }
}
