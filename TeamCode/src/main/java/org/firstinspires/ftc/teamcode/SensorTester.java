package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 11/22/2016.
 */

@Autonomous(name = "Sensor tester", group = "diagnostic")
public class SensorTester extends LinearBase {

    int sensorNum = 0;

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        waitForStart();

        while(opModeIsActive())
        {
            changeLightOn();
        }
    }

    public void changeLightOn()
    {
        red        .enableLed(false);
        blue       .enableLed(false);
        bottomLeft .enableLed(false);
        bottomRight.enableLed(false);

        if(sensorNum < 4)
            sensorNum += 1;
        else
            sensorNum = 0;

        switch(sensorNum)
        {
            case 1:
                red.enableLed(true);
                break;
            case 2:
                blue.enableLed(true);
                break;
            case 3:
                bottomLeft.enableLed(true);
                break;
            case 4:
                bottomLeft.enableLed(true);
                break;

        }
    }
}
