package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/11/2017.
 */

@Autonomous(name = "Shoot and Beacon, RED SIDE", group ="auto")
public class ShootAndBeaconRed extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        Thread.sleep(100);

        front.enableLed(true);

        moveShootMove(-0.75, 1440 * 2, 1440 * 1.5);

        turn(0.45, 45);

        move(MAX_MOVE_SPEED * -1, 1440*2.5);

        turn(0.45, -132);

        pressAndTest(MAX_MOVE_SPEED, 1440 * -2.8, Color.BLUE);

        move(-MAX_MOVE_SPEED, 1440*2);

        turn(0.45, 80); // DONT FALL OVER ROBOT
    }
}
