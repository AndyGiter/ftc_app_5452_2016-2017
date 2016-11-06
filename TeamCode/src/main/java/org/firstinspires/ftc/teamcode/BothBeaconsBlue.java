package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mlowery2 on 11/6/2016.
 */

@Autonomous(name = "BBBS, no shoot", group = "autonomous") // Both beacons blue side no shoot
public class BothBeaconsBlue extends LinearOpMode{

    Servo rightBp;
    Servo leftBp;

    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    ColorSensor red;    // The red side color sensor
    ColorSensor blue;   // The blue side color sensor
    ColorSensor bottom; // The color sensor on the bottom (for detecting lines.)

    I2cAddr i2cAddrRed    = I2cAddr.create8bit(0x3c); // If you replace a color sensor make sure to set the I2C address to the right one
    I2cAddr i2cAddrBlue   = I2cAddr.create8bit(0x4c); // Also, port 0 for the I2C sensors doesnt work
    I2cAddr i2cAddrBottom = I2cAddr.create8bit(0x5c);

    float hsvValuesRed[]    = {0F, 0F, 0F};
    float hsvValuesBlue[]   = {0F, 0F, 0F};
    float hsvValuesBottom[] = {0F, 0F, 0F};

    final int side = Color.BLUE;

    public void runOpMode() throws InterruptedException
    {
        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);

        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBp = hardwareMap.servo.get("right");
        leftBp  = hardwareMap.servo.get("left");

        red = hardwareMap.colorSensor.get("red");
        blue = hardwareMap.colorSensor.get("blue");
        bottom = hardwareMap.colorSensor.get("bottom");

        red.setI2cAddress(i2cAddrRed);
        blue.setI2cAddress(i2cAddrBlue);
        bottom.setI2cAddress(i2cAddrBottom);

        red.enableLed(true);
        blue.enableLed(true);
        bottom.enableLed(true);

        waitForStart();

        testAndPress(blue, leftBp, side); // make sure to test
    }

    public void testAndPress(ColorSensor sensor, Servo servo, int side) throws InterruptedException
    {
        final double changeBy = 0.01;
        final long waitTime = 100;

        double pos = 0.5;

        while(colorSeen(sensor) == 0)
        {
            servo.setPosition(pos);

            if(side == Color.BLUE)
                pos += changeBy; // TODO: make sure this increments when on the blue side (as opposed to decrement)

            else
                pos -= changeBy;

            if(pos == 1 || pos == 0) {
                telemetry.addData("Error: ", "Went to limit and didn't sense anything");
                telemetry.update();
                return;
            }

            Thread.sleep(waitTime);
        }

        if(colorSeen(sensor) == Color.GREEN) // to test to see if its fucked
        {
            telemetry.addData("Error: ", "Color seen was green. whaaaaat");
            telemetry.update();
            return;
        }
        else if(colorSeen(sensor) == side) // if the color seen is the side we're on, go all the way in
        {
            if(Math.abs(pos) == pos) // is positive
                servo.setPosition(1); // go all the way in

            else
                servo.setPosition(-1);
        }
        else // if it is not, go to the other side
        {
            if(Math.abs(pos) == pos) // is positive, we see a color thats not our side
                servo.setPosition(-1); // go to the other side

            else
                servo.setPosition(1);

        }

    }

    public void updateHsv()
    {
        Color.RGBToHSV(red.red() * 8,    red.green() * 8,    red.blue() * 8,    hsvValuesRed);
        Color.RGBToHSV(blue.red() * 8,   blue.green() * 8,   blue.blue() * 8,   hsvValuesBlue);
        Color.RGBToHSV(bottom.red() * 8, bottom.green() * 8, bottom.blue() * 8, hsvValuesBottom);
    }

    public int colorSeen(ColorSensor sensor) // RGB only
    {
        if(sensor.red() > sensor.blue() && sensor.red() > sensor.green())
            return Color.RED;

        else if(sensor.blue() > sensor.green() && sensor.blue() > sensor.red())
            return Color.BLUE;

        else if(sensor.green() > sensor.red() && sensor.green() > sensor.blue())
            return Color.GREEN;

        else
            return 0; // Technically this is color.TRANSPARENT but that's never going to be usefull and I need a way to return an error
    }
}
