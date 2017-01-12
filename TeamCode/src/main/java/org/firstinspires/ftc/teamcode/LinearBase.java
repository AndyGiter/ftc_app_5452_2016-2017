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
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 11/8/2016.
 *
 * I2C address checklist:
 *
 * Color Sensors // Make sure to check these
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

    private I2cAddr i2cAddrFront    = I2cAddr.create8bit(0x4c);

    float hsvValuesFront[]    = {0F, 0F, 0F};

    ModernRoboticsI2cGyro gyro; // Default I2C address 0x20

    enum Direction {LEFT, RIGHT, FORWARD, BACKWARD};
    enum Side {RED, BLUE};

    DcMotor.RunMode defualtRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    private final float DEADZONE = 0.200f;

    boolean verbose = false;

    final double MAX_MOVE_SPEED = 0.8;
    final double MAX_TURN_SPEED = 0.45;

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

        shooter = hardwareMap.dcMotor.get("snail"); // See if there is a need to reverse these.
        collector = hardwareMap.dcMotor.get("feed");

        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        right1.setMode(defualtRunMode);
        right2.setMode(defualtRunMode);
        left2.setMode(defualtRunMode);
        left1.setMode(defualtRunMode);

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Make sure to set to run to position once a solution is

        //Color Sensor Setup
        front = hardwareMap.colorSensor.get("front");

        front.setI2cAddress(i2cAddrFront);

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
        if(Math.abs(speed) == -1*speed && Math.abs(distance) == distance) // If speed isnegative and distance is positive
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

    public void pressAndTest(double speed, double distanceToWall, int colorWanted) throws InterruptedException
    {
        final double WAIT_BEFORE_MOVE = 5; // set to something like 4000 because of the time needed before pressing the button again
        final double MOVE_BACK_DIST = 1440*0.5 * -1;
        final double SECOND_PRESS_DIST = 1440 * 0.7 * -1;
        final int WAIT_BEFORE_READ = 300;

        front.enableLed(false);

        double startTime = getRuntime();

        move(speed, -1*distanceToWall); // move twords wall

        double pressTime = getRuntime();

        sleep(WAIT_BEFORE_READ);

        if(front.red() == 0 && front.blue() == 0) // no need to test green because its useless :(
        {
            telemetry.addData("Warning", "Not close enough to sense beacon color");
            telemetry.update();
        }

        else if(front.red() == 256 && front.blue() == 256)
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

    public void turnBackTo(double maxSpeed, int deg)
    {

    }

    public void shoot() throws InterruptedException
    {
        shooter.setPower(0.5);
        Thread.sleep(500);
        shooter.setPower(0);
    }

    public void moveShootMove(double speed, double totalDist, double distBeforeShoot) throws InterruptedException
    {
        if(distBeforeShoot != 0)
            move(speed, distBeforeShoot);

        shooter.setPower(-0.5);
        Thread.sleep(750);
        shooter.setPower(0);

        move(speed, totalDist - distBeforeShoot);
    }

    public void gyroTelemetry(ModernRoboticsI2cGyro sensorGyro)
    {
        telemetry.addData("Gyro: ", sensorGyro.getIntegratedZValue() + "");
        telemetry.addData("Gyro Heading: ", sensorGyro.getHeading() + "");
    }

    public void rangeTelemety(ModernRoboticsI2cRangeSensor rangeSensor)
    {
        telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", rangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM)); // what to use
    }

    public void colorTelemetry(ColorSensor color, float[] hsvVals)
    {
        telemetry.addData("Red", color.red());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Green", color.green());
        telemetry.addData("Alpha", color.alpha());
        telemetry.addData("HSV", hsvVals);
    }
}
