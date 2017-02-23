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

        moveShootMove(-0.5, 1440*1.15, 1440*1.15);

        turn(0.3, 45); // turn left 45

        move(-0.7, 1440 * 1.3); // the diag

        turn(0.45, -133); // turn to face the button

        pressAndTest(0.4, Color.RED);

        move(-0.5, 1440*2.5);
    }
}

/*
        moveShootMove(-MAX_MOVE_SPEED, 1440 * 2, 1440 * 2); // shoot

        turn(MAX_TURN_SPEED, 45); // turn left 45

        move(MAX_MOVE_SPEED * -1, 1440*4); // the diag

        turn(MAX_TURN_SPEED, -140); // turn to face the button

        pressAndTest(MAX_MOVE_SPEED, 1440 * -1.8, Color.RED); // press

        move(-MAX_MOVE_SPEED, 1440*4); // Move back into the cap ball

        turn(MAX_TURN_SPEED, -80); // DONT FALL OVER ROBOT (bump)
 */
