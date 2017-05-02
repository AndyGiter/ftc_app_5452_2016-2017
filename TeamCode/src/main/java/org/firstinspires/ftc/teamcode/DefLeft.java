package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 4/22/2017.
 */

@Autonomous(name = "Defencive (Left Turn)", group ="auto")
public class DefLeft extends LinearBase {

    private final double WAIT_BEFORE_CROSS = 10.5;

    public void runOpMode()
    {
        initialise(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();
        sleep(100);

        double start = getRuntime();

        moveShootMove(-0.3, 1440 * 1.8, 1440 * 1.8);

        sleep(250);

        turn(90);

        while(getRuntime()-start < WAIT_BEFORE_CROSS){} // Make sure 10 seconds have passed

        move(-1440*1.35);
    }
}
