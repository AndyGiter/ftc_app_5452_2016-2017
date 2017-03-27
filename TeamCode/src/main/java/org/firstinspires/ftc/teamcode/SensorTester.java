package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 3/22/2017.
 */

@Autonomous(name = "Range Sensor Testing", group = "Testing")
public class SensorTester extends LinearBase{

    public void runOpMode() throws InterruptedException
    {
        initAndWait(DcMotor.RunMode.RUN_TO_POSITION, true);

        while(opModeIsActive())
        {
            telemetry.addData("cm", "%.2f cm", frontRange.getDistance(DistanceUnit.CM));
            telemetry.addData("cm", "%.2f cm", sonarRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
