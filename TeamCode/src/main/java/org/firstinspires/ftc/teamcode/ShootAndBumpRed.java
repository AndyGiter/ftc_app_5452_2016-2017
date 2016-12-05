package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by mlowery2 on 11/7/2016.
 */
@Autonomous(name = "Shoot and Bump, RED SIDE", group ="testing")
public class ShootAndBumpRed extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize();
        waitForStart();
        Thread.sleep(100);

        moveShootMove(0.75, 1440*1.4, 0);

        turn(0.75, 1440 * (2.0 / 3.0), Direction.RIGHT);

        move(0.75, 1440 * 2);

        turn(0.75, 1440 * (4.0 / 3.0), Direction.LEFT);
        move(0.75, 1440 * 3);
        move(-0.75, 1440 * 0.5);
    }

    public void shoot() throws InterruptedException
    {
        cannon.setPosition(0);
        Thread.sleep(200);
    }

    public void moveShootMove(double speed, double totalDist, double distBeforeShoot) throws InterruptedException
    {
        if(distBeforeShoot != 0)
            move(speed, distBeforeShoot);

        cannon.setPosition(0);
        Thread.sleep(200);

        move(speed, totalDist - distBeforeShoot);
    }
}
