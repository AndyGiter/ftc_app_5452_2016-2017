package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

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

public class ColorTesting extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(DcMotor.RunMode.RUN_TO_POSITION, true);
        front.enableLed(true); // this is just because we're using a printout of red & blue instead of the beacon
        waitForStart();
        sleep(100);

        while(opModeIsActive())
        {
            Color.RGBToHSV(front.red(), front.blue(), front.blue(), hsvValuesFront);

            colorTelemetry(front, hsvValuesFront);
            telemetry.update();
        }
    }
}
