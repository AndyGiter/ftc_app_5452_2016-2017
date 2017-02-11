package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 2/10/2017.
 */

@Autonomous(name = "Opposing Cap TWO BALLS BLUE SIDE", group ="auto")
public class OpposingCapBlue extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        Thread.sleep(100);

        double start = getRuntime();

        moveShootMove(-0.75, 1440 * 3.5, 1440 * 3.5);

        turn(1, 45);

        while(getRuntime()-start <= 10 && opModeIsActive()) {}

        move(MAX_MOVE_SPEED, 1440 * -2.25);


    }
}
