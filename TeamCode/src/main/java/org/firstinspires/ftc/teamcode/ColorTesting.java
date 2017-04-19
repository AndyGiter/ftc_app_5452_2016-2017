package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.ColorSensor;
/**
 * Created by mlowery2 on 11/1/2016.
 *
 * This program is made to test if the color sensor can detect the color of the beacon in front of it while moving forward on a servo ar.
 */
@TeleOp(name = "Color Sensor Testing", group = "Testing")
@Disabled
public class ColorTesting extends LinearBase {

    private boolean ledState = true;

    public void runOpMode() throws InterruptedException
    {
        initAndWait(DcMotor.RunMode.RUN_TO_POSITION, true);
        frontColor.enableLed(ledState);

        while(opModeIsActive())
        {

            if (gamepad1.a)
            {
                ledState = !ledState;
                frontColor.enableLed(ledState);
                Thread.sleep(300);
            }

            colorTelemetry(frontColor);
            telemetry.update();
            Thread.sleep(50);
        }
    }
}
