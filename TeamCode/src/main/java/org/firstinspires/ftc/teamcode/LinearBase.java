package org.firstinspires.ftc.teamcode;

// Basic Needed stuff (some needed in the program that this will implement.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Sensors
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

 // Moving parts
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;

 // Misc.
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 11/8/2016.
 *
 * I2C address checklist:
 *
 * Color Sensors
 * front: 0x4c
 *
 * Gyro Sensor
 * gyro: 0x20 // Default
 */
public abstract class LinearBase extends LinearOpMode{

    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    DcMotor shooter;
    DcMotor collector;

    ColorSensor front;

    DigitalChannel touch;

    private I2cAddr i2cAddrFront = I2cAddr.create8bit(0x4c);

    float hsvValuesFront[]    = {0F, 0F, 0F};

    ModernRoboticsI2cGyro gyro; // Default I2C address 0x20

    enum Direction {LEFT, RIGHT, FORWARD, BACKWARD};

    DcMotor.RunMode defualtRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    private final float DEADZONE = 0.200f;

    boolean verbose = false;

    final double MAX_MOVE_SPEED = 0.8;
    final double MAX_TURN_SPEED = 0.45;

    public boolean running = false; // for if the shoot thread is running

    public void initalize() throws InterruptedException
    {
        double start = getRuntime();
        if(verbose){telemetry.addData("Done: ","Starting init"); telemetry.update();}

        // Gyro (this comes first so we can do other things, like initalizing other things, while this calibrates.)
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();

        // Touch Sensor Setup
        touch = hardwareMap.get(DigitalChannel.class, "touch"); // touch.getState() gets the state
        touch.setMode(DigitalChannelController.Mode.INPUT);

        // Motor Setup
        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        shooter = hardwareMap.dcMotor.get("snail"); // TODO: Set this to be reversed and fix all the code that is broken because of that
        collector = hardwareMap.dcMotor.get("feed");

        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.sleep(150);

        right1.setMode(defualtRunMode);
        right2.setMode(defualtRunMode);
        left2.setMode(defualtRunMode);
        left1.setMode(defualtRunMode);

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //Color Sensor Setup
        front = hardwareMap.colorSensor.get("front");

        front.setI2cAddress(i2cAddrFront);

        // Light needs to be off for use, but is on as a visual example that the color sensor is reading commands and that the robot is still initalizing (even though there are other clues)
        front.enableLed(true);

        // Setting the deadzone for the gamepads
        gamepad1.setJoystickDeadzone(DEADZONE);
        gamepad2.setJoystickDeadzone(DEADZONE);

        if(verbose){telemetry.addData("Done: ","Everything but gyro. Took " + (getRuntime()-start) + " seconds"); telemetry.update();}

        while (!isStopRequested() && gyro.isCalibrating())  { // Make sure that the gyro is calibrated
            Thread.sleep(50);
        }


        if(verbose){telemetry.addData("Done: ","Initalizing. Took " + (getRuntime()-start) + " seconds"); telemetry.update();}
        else{telemetry.addData("Done", "Initalizing"); telemetry.update();}

        front.enableLed(false);

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
        //TODO: figure that out
        if(Math.abs(speed) == -1*speed && Math.abs(distance) == distance) // If speed is negative and distance is positive
        {distance *= -1;}
        else if(Math.abs(distance) == distance*-1 && Math.abs(speed) == speed) // If distance is negative and speed is positive
        {speed *= -1;}

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

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION) // last thing to do
        {
            right1.setMode(defualtRunMode);
            right2.setMode(defualtRunMode);
            left2.setMode(defualtRunMode);
            left1.setMode(defualtRunMode);
        }

        Thread.sleep(300);
    }

    // This is just kept around as a brute force turning method if anything goes down
    @Deprecated
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

        else if(d == Direction.LEFT)
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

        else
        {
            telemetry.addData("Error", "Bad input, must be left or right");
            telemetry.update();
            return;
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

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION) // last thing to do
        {
            right1.setMode(defualtRunMode);
            right2.setMode(defualtRunMode);
            left2.setMode(defualtRunMode);
            left1.setMode(defualtRunMode);
        }

        Thread.sleep(300);
    }

    /*
     * If no direction is given. This is done mainly so I don't have to go back and update old code that doesnt use the gyro and uses encoder values
     * Negative deg value turns right
     * Positive is left
     */
    public void turn(double maxSpeed, int deg) throws InterruptedException
    {
        final int TURN_RANGE = 6;
        int targetHeading = gyro.getIntegratedZValue() + deg;

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // While the gyro is not within the range (TURN_RANGE)
        while(!(gyro.getIntegratedZValue() <= targetHeading+(TURN_RANGE/2.0) && gyro.getIntegratedZValue() >= targetHeading-(TURN_RANGE/2.0)) && opModeIsActive())
        {

            if(gyro.getIntegratedZValue() > targetHeading) // Right turn
            {
                left1.setPower(maxSpeed);
                left2.setPower(maxSpeed);
                right1.setPower(-1 * maxSpeed);
                right2.setPower(-1 * maxSpeed);
            }

            else //then its a left turn
            {
                left1.setPower(-1 * maxSpeed);
                left2.setPower(-1 * maxSpeed);
                right1.setPower(maxSpeed);
                right2.setPower(maxSpeed);
            }

            if(verbose)
            {
                telemetry.addData("Gyro target: ", targetHeading+"");
                telemetry.addData("Gyro heading: ", gyro.getIntegratedZValue()+"");
                telemetry.addData("Robot speed: ", maxSpeed+"");
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



    public void pressAndTest(double speed, double distanceToWall, int colorWanted) throws InterruptedException
    {
        final double WAIT_BEFORE_MOVE = 5; // TODO: Test to see what the smallest value for this is
        final double MOVE_BACK_DIST = 1440*0.5 * -1;
        final double SECOND_PRESS_DIST = 1440 * 0.7 * -1;
        final int WAIT_BEFORE_READ = 300;

        front.enableLed(false);

        double startTime = getRuntime();

        move(speed, -1 * distanceToWall); // move twords wall

        double pressTime = getRuntime();

        sleep(WAIT_BEFORE_READ);

        if(front.red() == 0 && front.blue() == 0) // no need to test green because its useless :(
        {
            telemetry.addData("Warning", "Not close enough to sense beacon color");
            telemetry.update();
        }

        else if(front.red() == 255 && front.blue() == 255)
        {
            telemetry.addData("Error", "Color sensor is fuckn broken again");
            telemetry.update();
        }

        else if((front.red() > front.blue() && colorWanted == Color.BLUE) || (front.blue() > front.red() && colorWanted == Color.RED))
        {

            if(verbose)
            {
                colorTelemetry(front, hsvValuesFront);
                telemetry.update();
            }

            move(speed, MOVE_BACK_DIST); // move back from wall

            while(getRuntime()-pressTime < WAIT_BEFORE_MOVE && opModeIsActive()){sleep(50);} // making sure to wait enough time before pressing again

            move(speed, -1*SECOND_PRESS_DIST); // move back in, hitting the button
        }
        else if(verbose)
        {
                colorTelemetry(front, hsvValuesFront);
                telemetry.update();
        }

        move(speed, distanceToWall); // move back to position started in
    }

    public void turnBackTo(double maxSpeed, int deg) // TODO: Make this
    {

    }

    public void shoot() throws InterruptedException // TODO: Make this better (so it resets but without wasting time)
    {
        shooter.setPower(0.5);
        Thread.sleep(500);
        shooter.setPower(0);
    }

    public void shootThreaded() // needs to be tested
    {
        final int WAIT_TIME = 200; // in miliseconds

        if(!running)
        {
            new Thread(new Runnable() {
                public void run() {
                    running = true;

                    shooter.setPower(0.9);

                    sleep(WAIT_TIME);

                    while(!touch.getState()){}

                    shooter.setPower(0);

                    running = false;
                }
            }).start();
        }
    }

    public void moveShootMove(double speed, double totalDist, double distBeforeShoot) throws InterruptedException // TODO: Make this better (so it resets but without wasting time)
    {
        if(distBeforeShoot != 0)
            move(speed, distBeforeShoot);

        shooter.setPower(0.5);
        Thread.sleep(750);
        shooter.setPower(0);

        if(distBeforeShoot < totalDist)
            move(speed, totalDist - distBeforeShoot);
    }

    public void gyroTelemetry(ModernRoboticsI2cGyro sensorGyro)
    {
        telemetry.addData("Gyro: ", sensorGyro.getIntegratedZValue() + "");
        telemetry.addData("Gyro Heading: ", sensorGyro.getHeading() + "");
    }

    // Not in use (and I dont think it will be again) but just if we need it, this is still here
    public void rangeTelemety(ModernRoboticsI2cRangeSensor rangeSensor)
    {
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM)); // what to use
    }

    public void colorTelemetry(ColorSensor color)
    {
        telemetry.addData("Red", color.red());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Green", color.green());
        telemetry.addData("Alpha", color.alpha());
    }

    public void colorTelemetry(ColorSensor color, float[] hsvVals)
    {
        telemetry.addData("HSV", hsvVals);
        colorTelemetry(color);
    }
}
