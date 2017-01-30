package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/23/2017.
 */

@Autonomous(name = "HOW DO ENCODER?", group ="testing")
public class EncoderTester extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(true);
        collector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //collector.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        Thread.sleep(100);

        collector.setTargetPosition(collector.getTargetPosition() + 1440);
        collector.setPower(0.9);

        while(collector.isBusy() && opModeIsActive()){Thread.sleep(50);}

        collector.setPower(0);

        Thread.sleep(100);
    }
}
