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
        front.enableLed(true);

        moveShootMove(-0.5, 1440 * 1.15, 1440 * 1.15);

        turn(0.3, -45); // turn right 45

        move(-0.7, 1440 * 1.7); // the diag

        turn(0.45, 133); // turn to face the button

        pressAndTest(0.4, Color.BLUE);

        move(-0.5, 1440 * 2.5);
    }
}
/*
        moveShootMove(-MAX_MOVE_SPEED, 1440 * 2, 1440 * 1.9); // shoot

        turn(MAX_TURN_SPEED, -45); // turn right 45

        move(MAX_MOVE_SPEED * -1, 1440 * 3.45); // the diag

        turn(MAX_TURN_SPEED, 130); // turn to press the button

        pressAndTest(MAX_MOVE_SPEED, 1440 * -2.0, Color.BLUE); // press

        move(-MAX_MOVE_SPEED, 1440 * 2.25); // move back to hit the ball

        turn(MAX_TURN_SPEED, 80); // DONT FALL OVER ROBOT (bump)
 */
