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

public class ColorTesting extends LinearOpMode {

    ColorSensor red;    // The red side color sensor
    ColorSensor blue;   // The blue side color sensor
    ColorSensor bottom; // The color sensor on the bottom (for detecting lines.)

    Servo rightBp;
    Servo leftBp;

    double servoPos = 0.5; // Init position

    boolean LEDon = false;

    float hsvValuesRed[]   = {0F,0F,0F};
    float hsvValuesBlue[]   = {0F,0F,0F};
    float hsvValuesBottom[] = {0F,0F,0F};

    boolean wait = false;
    final double changeBy = 0.01;
    final long waitTime = 100; // in ms

    public void runOpMode() throws InterruptedException
    {
        //red    = hardwareMap.colorSensor.get("red");
        blue   = hardwareMap.colorSensor.get("blue"); // TODO: Update to include the I2C address stuff so it will work
        bottom = hardwareMap.colorSensor.get("bottom");

        //red   .enableLed(LEDon);
        blue  .enableLed(LEDon);
        bottom.enableLed(LEDon);

        rightBp = hardwareMap.servo.get("right");
        leftBp  = hardwareMap.servo.get("left");

        rightBp.scaleRange(0.1, 0.76); // Retest these values before the tournament
        leftBp .scaleRange(0, 0.57);    // Retest these values before the tournament

        rightBp.setPosition(servoPos);
        leftBp .setPosition(servoPos);

        waitForStart();
        updateHsv();

        while(opModeIsActive())
        {
            if(gamepad1.x) // left
            {
                if(servoPos < 1)
                {
                    servoPos += changeBy;
                    wait = true;
                }
            }
            else if(gamepad1.y) // middle
            {
                servoPos = 0.5;
                wait = true;
            }
            else if(gamepad1.b) // right
            {
                if(servoPos > 0)
                {
                    servoPos -= changeBy;
                    wait = true;
                }
            }
            else if(gamepad1.a)
            {
                LEDon = !LEDon;
                wait = true;

                blue  .enableLed(LEDon);
                bottom.enableLed(LEDon);
            }

            leftBp.setPosition(servoPos);
            rightBp.setPosition(servoPos);

            if(wait)
            {
                wait = false;
                Thread.sleep(waitTime);
            }

            updateHsv();

            telemetry.addData("Color Sensor: ", "Blue");
            telemetry.addData("Red  ", blue.red());
            telemetry.addData("Green", blue.green());
            telemetry.addData("Blue ", blue.blue());
            telemetry.addData("Hue", hsvValuesRed[0]);
            telemetry.addData("Servo Pos: ", servoPos);
            telemetry.addData("Light?", LEDon);
            telemetry.update();
        }
    }

    public void updateHsv()
    {
        //Color.RGBToHSV(red.red() * 8,    red.green() * 8,    red.blue() * 8,    hsvValuesRed);
        Color.RGBToHSV(blue.red() * 8,   blue.green() * 8,   blue.blue() * 8,   hsvValuesBlue);
        Color.RGBToHSV(bottom.red() * 8, bottom.green() * 8, bottom.blue() * 8, hsvValuesBottom);
    }
}
