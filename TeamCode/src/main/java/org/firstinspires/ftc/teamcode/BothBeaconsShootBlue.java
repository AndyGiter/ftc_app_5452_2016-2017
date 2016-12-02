package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 12/1/2016.
 *
 * Id like to ask god for help to make sure this shit works
 */
@Autonomous(name="BBS (Blue)", group="testing")
public class BothBeaconsShootBlue extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(true);
        waitForStart();

        pressAndTest(leftBp, blue, 0, 1, Color.BLUE);
        telemetry.addData("Distance", rangeLeft.getDistance(DistanceUnit.CM)+"");
        telemetry.update();

        while(opModeIsActive()){Thread.sleep(50);}
    }
}
