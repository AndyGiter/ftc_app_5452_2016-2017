package org.firstinspires.ftc.teamcode;

// Basic Needed stuff (some needed in the program thiat this will implement.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Sensors
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

 // Moving parts
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

 // Misc.
import com.qualcomm.robotcore.hardware.I2cAddr; // TODO: Make sure that all I2c addresses are set correctly on all sensors and record what they are in this program.
import com.qualcomm.robotcore.util.Range;

import android.graphics.Color;
/**
 * Created by mlowery2 on 11/8/2016.
 *
 * I2C address checklist:
 *
 * Color Sensors
 * red:         0x3c // Default
 * blue:        0x4c
 * bottomLeft:  0x5c
 * bottomRight: 0x6c
 *
 * Range Sensors
 * left:  0x28 // Default
 * right: 0x26
 *
 * Gyro Sensor
 * gyro: 0x20 // Default
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
    ColorSensor bottomLeft; // The color sensor on the bottom (for detecting lines.)
    ColorSensor bottomRight; // The color sensor on the bottom (for detecting lines.)

    private I2cAddr i2cAddrRed    = I2cAddr.create8bit(0x3c); // If you replace a color sensor make sure to set the I2C address to the right one
    private I2cAddr i2cAddrBlue   = I2cAddr.create8bit(0x4c); // Also, port 0 for the I2C sensors doesnt work
    private I2cAddr i2cAddrBottomLeft = I2cAddr.create8bit(0x5c);
    private I2cAddr i2cAddrBottomRight = I2cAddr.create8bit(0x6c);

    float hsvValuesRed[]    = {0F, 0F, 0F};
    float hsvValuesBlue[]   = {0F, 0F, 0F};
    float hsvValuesBottomLeft[] = {0F, 0F, 0F};
    float hsvValuesBottomRight[] = {0F, 0F, 0F};

    ModernRoboticsI2cGyro gyro; // Default I2C address 0x20

    ModernRoboticsI2cRangeSensor rangeLeft;
    ModernRoboticsI2cRangeSensor rangeRight;

    private I2cAddr i2cAddrRangeLeft = I2cAddr.create8bit(0x28);
    private I2cAddr i2cAddrRangeRight = I2cAddr.create8bit(0x26);

    enum Direction {LEFT, RIGHT};
    enum Side {RED, BLUE};

    DcMotor.RunMode defualtRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    private final float DEADZONE = 0.200f;

    boolean verbose = false;
    /* Template for using telemetry in a function

    if(verbose){telemetry.addData("",""); telemetry.update();}

    */

    public void initalize() throws InterruptedException
    {
        double start = getRuntime();
        if(verbose){telemetry.addData("Done: ","Starting init"); telemetry.update();}

        //Gyro (this comes first so we can do other things, like initalizing other things, while this calibrates.)
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

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
        bottomLeft = hardwareMap.colorSensor.get("bottomLeft");
        bottomRight = hardwareMap.colorSensor.get("bottomRight");

        red.setI2cAddress(i2cAddrRed);
        blue.setI2cAddress(i2cAddrBlue);
        bottomLeft.setI2cAddress(i2cAddrBottomLeft);
        bottomRight.setI2cAddress(i2cAddrBottomRight);

        red.enableLed(true);
        blue.enableLed(true);
        bottomLeft.enableLed(true);
        bottomRight.enableLed(true);

        // Range sensor setup
        rangeLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeLeft");
        rangeRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeRight");

        rangeLeft.setI2cAddress(i2cAddrRangeLeft);
        rangeRight.setI2cAddress(i2cAddrRangeRight);

        // Setting the deadzone for the gamepads
        gamepad1.setJoystickDeadzone(DEADZONE);
        gamepad2.setJoystickDeadzone(DEADZONE);

        if(verbose){telemetry.addData("Done: ","Everything but gyro. Took " + (getRuntime()-start) + " seconds"); telemetry.update();}
        while (!isStopRequested() && gyro.isCalibrating())  { // Make sure that the gyro is calibrated
            Thread.sleep(50);
        }

        if(verbose){telemetry.addData("Done: ","Initalizing. Took " + (getRuntime()-start) + " seconds"); telemetry.update();}

    }
    public void initalize(DcMotor.RunMode newDefualtRunMode) throws InterruptedException
    {
        defualtRunMode = newDefualtRunMode;
        initalize();
    }

    public void initalize(boolean newVerbose) throws InterruptedException
    {
        verbose = newVerbose;
        initalize();
    }
    public void initalize(DcMotor.RunMode newDefualtRunMode, boolean newVerbose) throws InterruptedException
    {
        defualtRunMode = newDefualtRunMode;
        verbose = newVerbose;
        initalize();
    }

    public void updateHsv()
    {
        Color.RGBToHSV(red.red() * 8,    red.green() * 8,    red.blue() * 8,    hsvValuesRed);
        Color.RGBToHSV(blue.red() * 8,   blue.green() * 8,   blue.blue() * 8,   hsvValuesBlue);
        Color.RGBToHSV(bottomLeft.red() * 8, bottomLeft.green() * 8, bottomLeft.blue() * 8, hsvValuesBottomLeft);
        Color.RGBToHSV(bottomRight.red() * 8, bottomRight.green() * 8, bottomRight.blue() * 8, hsvValuesBottomRight);
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

    public void move(double speed, double distance) throws InterruptedException // TODO: Integrate the gyro somehow
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
            left1.setPower(-1 * speed);
            left2.setPower(-1 * speed);

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

    /*
     * If no direction is given. This is done mainly so I don't have to go back and update old code that doesnt use the gyro and inputs encoder values
     * Negative deg value turns right
     * Positive is left
     */
    public void turn(double maxSpeed, int deg) throws InterruptedException
    {
        final int TURN_RANGE = 10;
        int targetHeading = gyro.getIntegratedZValue() + deg;

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER) {
            right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // While the gyro is not within the range (TURN_RANGE)
        while(!(gyro.getIntegratedZValue() <= targetHeading+(TURN_RANGE/2.0) && gyro.getIntegratedZValue() >= targetHeading-(TURN_RANGE/2.0)) && opModeIsActive()) // make sure this works
        {
            /**
             * These next two lines of code are kind of messy. The point is to decrease the speed over the turing distance.
             * There is probably a better way to do this, but my math and programming skills limit me.
             *
             * Hopefully it works
             */
        /*
            double percentDone = Math.abs((targetHeading - gyro.getIntegratedZValue())/deg);
            double speed = maxSpeed*decrease(percentDone);

            if(speed > maxSpeed || speed <0)
            {
                speed = maxSpeed;
                telemetry.addData("Error: ", "You did it wrong dingus");
                telemetry.update();
            }

            //speed = Range.clip(maxSpeed * (Math.abs(gyro.getIntegratedZValue() - targetHeading) / Math.abs(deg)), .01, maxSpeed); // coppied from brendan's code, thanks brendan

            */
            double speed = maxSpeed; // Uncoment if the thing above doesnt work

            if(gyro.getIntegratedZValue() > targetHeading) // Right turn
            {
                left1.setPower(speed);
                left2.setPower(speed);
                right1.setPower(-1*speed);
                right2.setPower(-1 * speed);
            }

            else //then its a left turn
            {
                left1.setPower(-1*speed);
                left2.setPower(-1*speed);
                right1.setPower(speed);
                right2.setPower(speed);
            }

            if(verbose)
            {
                telemetry.addData("Gyro target: ", targetHeading+"");
                telemetry.addData("Gyro headingL: ", gyro.getIntegratedZValue()+"");
                //telemetry.addData("Gyro percent done: ", percentDone);
                telemetry.addData("Robot speed: ", speed+"");
                telemetry.update();
            }
        }

        right1.setPower(0);
        right2.setPower(0);
        left1.setPower(0);
        left2.setPower(0);

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER) // last thing to do
        {
            right1.setMode(defualtRunMode);
            right2.setMode(defualtRunMode);
            left2.setMode(defualtRunMode);
            left1.setMode(defualtRunMode);
        }

        Thread.sleep(300);

        if(verbose){telemetry.addData("Done: ", "Turning. " + deg + " deg. Heading: " + gyro.getIntegratedZValue()); telemetry.update();}

    }

    private double decrease(double x) // just put it into wolfram alpha and see the beauty
    {return (Math.cos(3*x)+1)/2;}

    // This should be used more in testing code
    public void pressAndTest(Servo servo, ColorSensor sensor, double pressDistance)
    {

    }

    // This sould be used more in final code
    public void pressAndTest(Servo servo, ColorSensor sensor)
    {
        final double defaultPressDistance = 1; // Make sure to test that this is all the way extended
        pressAndTest(servo, sensor, defaultPressDistance);
    }

    public void alignToLine()
    {

    }

    public void moveCloserToWall(double inputMinDistance)
    {
        final double minDistance = 5; // CM
    }
}
