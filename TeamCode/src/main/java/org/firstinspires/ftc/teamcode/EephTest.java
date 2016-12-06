package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 12/6/2016.
 */
@Autonomous(name = "Material Test", group ="test")
public class EephTest extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        Thread.sleep(100);

        move(-30, 1440 * -2);
        move(30, 1440);
    }
}
