package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;
/**
 * Created by mlowery2 on 11/8/2016.
 */
public abstract class LinearBase extends LinearOpMode{
    Servo rightBp;
    Servo leftBp;
    Servo cannon;

    final double RBP_INIT = 0.5;
    final double LBP_INIT = 0.5;
    final double C_INIT   = 0.18;

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

    int side = Color.GREEN; // Default side

    enum Direction {LEFT, RIGHT};

    DcMotor.RunMode defualtRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    final float DEADZONE = 0.200f;

    boolean verbose = false; // TODO: Add to init function or dont idk

    public void initalize()
    {
        // Motor Setup
        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);

        right1.setMode(defualtRunMode);
        right2.setMode(defualtRunMode);
        left2.setMode(defualtRunMode);
        left1.setMode(defualtRunMode);

        // Servo Setup
        rightBp = hardwareMap.servo.get("right");
        leftBp  = hardwareMap.servo.get("left");
        cannon = hardwareMap.servo.get("cannon");

        rightBp.scaleRange(0.1, 0.76);
        leftBp.scaleRange(0, 0.57);

        rightBp.setPosition(RBP_INIT);
        leftBp.setPosition(LBP_INIT);
        cannon.setPosition(C_INIT);

        //Color Sensor Setup
        red = hardwareMap.colorSensor.get("red");
        blue = hardwareMap.colorSensor.get("blue");
        bottom = hardwareMap.colorSensor.get("bottom");

        red.setI2cAddress(i2cAddrRed);
        blue.setI2cAddress(i2cAddrBlue);
        bottom.setI2cAddress(i2cAddrBottom);

        red.enableLed(true);
        blue.enableLed(true);
        bottom.enableLed(true);

        gamepad1.setJoystickDeadzone(DEADZONE);
        gamepad2.setJoystickDeadzone(DEADZONE);

    }
    public void initalize(int newSide)
    {
        side = newSide;
        initalize();
    }
    public void initalize(DcMotor.RunMode newDefualtRunMode)
    {
        defualtRunMode = newDefualtRunMode;
        initalize();
    }
    public void initalize(int newSide, DcMotor.RunMode newDefualtRunMode)
    {
        side = newSide;
        defualtRunMode = newDefualtRunMode;
        initalize();
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

    public void move(double speed, double distance) throws InterruptedException
    {
        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //This if tests if the speed is negative and sets the distance to be negative.
        //I have no idea if this is needed when moving backwards
        if(Math.abs(speed) == -1*speed && Math.abs(distance) == distance) // if it is negative.
        {distance *= -1;}

        right1.setTargetPosition((int) (right1.getCurrentPosition() + distance));
        right2.setTargetPosition((int) (right2.getCurrentPosition() + distance));
        left1.setTargetPosition((int) (left1.getCurrentPosition() + distance));
        left2.setTargetPosition((int) (left2.getCurrentPosition() + distance));

        right1.setPower(speed);
        right2.setPower(speed);
        left1.setPower(speed);
        left2.setPower(speed);

        while(opModeIsActive() && right1.isBusy() &&
                                  right2.isBusy() &&
                                  left1.isBusy()  &&
                                  left2.isBusy()){
            Thread.sleep(50);

        }

        right1.setPower(0);
        right2.setPower(0);
        left1.setPower(0);
        left2.setPower(0);

        Thread.sleep(300);

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION) // last thing to do
        {
            right1.setMode(defualtRunMode);
            right2.setMode(defualtRunMode);
            left2.setMode(defualtRunMode);
            left1.setMode(defualtRunMode);
        }
    }

    public void turn(double speed, double distance, Direction d) throws InterruptedException
    {
        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(d == Direction.RIGHT) // if we want to turn right
        {
            right1.setTargetPosition((int) (right1.getCurrentPosition() + -1*distance));
            right2.setTargetPosition((int) (right2.getCurrentPosition() + -1*distance));
            left1.setTargetPosition((int) (left1.getCurrentPosition() + distance));
            left2.setTargetPosition((int) (left2.getCurrentPosition() + distance));

            right1.setPower(-1*speed);
            right2.setPower(-1*speed);
            left1.setPower(speed);
            left2.setPower(speed);
        }

        else
        {
            right1.setTargetPosition((int) (right1.getCurrentPosition() + distance));
            right2.setTargetPosition((int) (right2.getCurrentPosition() + distance));
            left1.setTargetPosition((int) (left1.getCurrentPosition() + -1*distance));
            left2.setTargetPosition((int) (left2.getCurrentPosition() + -1*distance));

            right1.setPower(speed);
            right2.setPower(speed);
            left1.setPower(-1*speed);
            left2.setPower(-1*speed);

        }

        while(opModeIsActive() && right1.isBusy() &&
                                  right2.isBusy() &&
                                  left1.isBusy()  &&
                                  left2.isBusy()){
            Thread.sleep(50);

        }

        right1.setPower(0);
        right2.setPower(0);
        left1.setPower(0);
        left2.setPower(0);

        Thread.sleep(300);

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION) // last thing to do
        {
            right1.setMode(defualtRunMode);
            right2.setMode(defualtRunMode);
            left2.setMode(defualtRunMode);
            left1.setMode(defualtRunMode);
        }
    }
}
