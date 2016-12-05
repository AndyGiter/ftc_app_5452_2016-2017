package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 12/1/2016.
 */

@Autonomous(name="Both Beacons Shoot RED", group="testing")
public class BothBeaconsShootRed extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(true);
        waitForStart();

        pressAndTest(rightBp, red, 0.8, 0, Color.RED);

        telemetry.addData("Distance", rangeRight.getDistance(DistanceUnit.CM) + ""); // ramge 12-15
        telemetry.update();

        while(opModeIsActive()){Thread.sleep(50);}
    }
}
