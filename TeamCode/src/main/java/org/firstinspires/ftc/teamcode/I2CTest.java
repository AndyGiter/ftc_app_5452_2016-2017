package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by the coolest guy on the team on 11/6/2016.
 */

@Autonomous(name = "I2C Test (please work)", group = "Testing")
public class I2CTest extends LinearOpMode {

    ColorSensor red;    // The red side color sensor
    ColorSensor blue;   // The blue side color sensor
    ColorSensor bottom; // The color sensor on the bottom (for detecting lines.)

    I2cAddr i2cAddrRed    = I2cAddr.create8bit(0x3c); // If you replace a color sensor make sure to set the I2C address to the right one
    I2cAddr i2cAddrBlue   = I2cAddr.create8bit(0x4c); // Also, port 0 for the I2C sensors doesnt work
    I2cAddr i2cAddrBottom = I2cAddr.create8bit(0x5c);

    float hsvValuesRed[]    = {0F, 0F, 0F};
    float hsvValuesBlue[]   = {0F, 0F, 0F};
    float hsvValuesBottom[] = {0F, 0F, 0F};

    int time = 0;

    public void runOpMode() throws InterruptedException {
        red = hardwareMap.colorSensor.get("red");
        blue = hardwareMap.colorSensor.get("blue");
        bottom = hardwareMap.colorSensor.get("bottom");

        red.setI2cAddress(i2cAddrRed);
        blue.setI2cAddress(i2cAddrBlue);
        bottom.setI2cAddress(i2cAddrBottom);

        waitForStart();

        while (opModeIsActive()) {
            time++;
            red.enableLed(false);
            blue.enableLed(false);
            bottom.enableLed(false);

            switch (time) {
                case 1:
                    red.enableLed(true);
                    break;
                case 2:
                    blue.enableLed(true);
                    break;
                case 3:
                    bottom.enableLed(true);
                    time = 0;
                    break;
            }

            updateHsv();
            telemetry.addData("Red Hue", hsvValuesRed[0]);
            telemetry.addData("Blue Hue", hsvValuesBlue[0]);
            telemetry.addData("Bottom Hue", hsvValuesBottom[0]);
            telemetry.update();

            Thread.sleep(250);
        }

    }

    public void updateHsv() {
        Color.RGBToHSV(red.red() * 8, red.green() * 8, red.blue() * 8, hsvValuesRed);
        Color.RGBToHSV(blue.red() * 8, blue.green() * 8, blue.blue() * 8, hsvValuesBlue);
        Color.RGBToHSV(bottom.red() * 8, bottom.green() * 8, bottom.blue() * 8, hsvValuesBottom);
    }
}

