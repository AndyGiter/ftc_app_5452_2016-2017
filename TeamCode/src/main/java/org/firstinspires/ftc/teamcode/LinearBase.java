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

// Misc.
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.Range;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This is the program that all other programs should extend (if they're ment to be run on the robot).
 * This has the initialisation functions, move functions, turn functions, and more. This program extends
 * LinearOpMode but does not have a runOpMode function because this class is not meant to be run, it
 * only has functions for classes that are meant to run on the robot.
 *
 * @author Max "Max the Sax" Lowery
 * @since 11-8-16
 * @version TODO: Add at the end of worlds
 */
public abstract class LinearBase extends LinearOpMode{

    DcMotor left1;
    DcMotor left2;
    DcMotor left3;
    DcMotor right1;
    DcMotor right2;
    DcMotor right3;

    DcMotor shooter;
    DcMotor collector;

    ColorSensor frontColor;

    private I2cAddr i2cAddrFrontColor = I2cAddr.create8bit(0x3c); // Default for color

    DigitalChannel touch;

    ModernRoboticsI2cGyro gyro; // Default I2C address 0x20

    private I2cAddr i2cAddrGyro = I2cAddr.create8bit(0x20);

    ModernRoboticsI2cRangeSensor frontRange; // The range sensor on the front of the robot

    private I2cAddr i2cAddrFrontRange = I2cAddr.create8bit(0x28);

    private DcMotor.RunMode defualtRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    //TODO: Investigate why that controller broke at supers and maybe change the deadzone system to just be set were ever the joystick is when init
    private final float DEADZONE = 0.200f;

    // If true then add extra logging
    boolean verbose = false;

    // These are just max speeds for autonomous, Teleop can go faster because it doesn't have to be as accurate and if there is a case where a fuse could blow adjustments can be made on the spot
    final double MAX_MOVE_SPEED = 0.5;
    final double MAX_TURN_SPEED = 0.5;

    final int END_WAIT = 200;

    public boolean running = false; // for if the shoot thread is running
    public boolean stopThread = false; // for use when the button is broken. Turn to true

    /**
     * The initialise method is used to initialise motors, servos, sensors, and the gamepads so that they all work
     *
     * @throws InterruptedException
     */
    public void initialise() throws InterruptedException
    {
        double start = getRuntime();

        telemetry.addData("Done: ", "Starting init"); telemetry.update();

        // Gyro (this comes first so we can do other things, like initialising other things, while this calibrates.)
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.setI2cAddress(i2cAddrGyro);
        gyro.calibrate();

        // Touch Sensor Setup
        touch = hardwareMap.get(DigitalChannel.class, "touch"); // touch.getState() gets the state
        touch.setMode(DigitalChannelController.Mode.INPUT);

        // Motor Setup
        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        left3  = hardwareMap.dcMotor.get("left3");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");
        right3 = hardwareMap.dcMotor.get("right3");

        shooter = hardwareMap.dcMotor.get("snail");
        collector = hardwareMap.dcMotor.get("feed");

        right1.setDirection(DcMotor.Direction.REVERSE);
        right3.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders(); // Resets the encoders and sets the mode to the default runmode

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //Color Sensor Setup
        frontColor = hardwareMap.colorSensor.get("front");

        frontColor.setI2cAddress(i2cAddrFrontColor);

        frontRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");

        frontRange.setI2cAddress(i2cAddrFrontRange);

        // Setting the deadzone for the gamepads
        gamepad1.setJoystickDeadzone(DEADZONE);
        gamepad2.setJoystickDeadzone(DEADZONE);

        if(verbose){telemetry.addData("Done: ","Everything but gyro. Took %.2f seconds", (getRuntime()-start)); telemetry.update();}

        while (!isStopRequested() && gyro.isCalibrating())  { // Make sure that the gyro is calibrated
            Thread.sleep(50);
        }


        if(verbose){telemetry.addData("Done: ","Initalizing. Took %.2f seconds", (getRuntime()-start)); telemetry.update();}
        else{telemetry.addData("Done", "Initalizing"); telemetry.update();}

        // When reading something that is emitting light, like the beacons, the LED on the color sensor
        // should be off. When trying to measure the color of something that isnt emitting light, turn
        // the LED on for the best measurements.
        frontColor.enableLed(false);
    }

    /**
     * For setting a different default runMode before initialising
     *
     * @param newDefualtRunMode A new default runMode for the drive motors, other motors cannot be changed
     * @throws InterruptedException
     */
    public void initialise(DcMotor.RunMode newDefualtRunMode) throws InterruptedException
    {
        defualtRunMode = newDefualtRunMode;
        initialise();
    }

    /**
     * For setting a different level of verbosity
     *
     * @param newVerbose True is more logging through telemetry, false is the most basic amount
     * @throws InterruptedException
     */
    public void initialise(boolean newVerbose) throws InterruptedException
    {
        verbose = newVerbose;
        initialise();
    }

    /**
     * For setting a different default runMode and a different level of verbosity
     *
     * @param newDefualtRunMode A new default runMode for the drive motors, other motors cannot be changed
     * @param newVerbose        True is more logging through telemetry, false is the most basic amount
     * @throws InterruptedException
     */
    public void initialise(DcMotor.RunMode newDefualtRunMode, boolean newVerbose) throws InterruptedException
    {
        defualtRunMode = newDefualtRunMode;
        verbose = newVerbose;
        initialise();
    }

    /**
     * To initialise and wait for start. When this was programmed we needed the servos to be initialised, but they could
     * not be initialised inside the 18' space (because of the limitations of the modern robotics servo controller).
     * We had this function to make sure that they did get initialised correctly but only after the game started.
     *
     * @param newDefault A new default runMode for the drive motors, other motors cannot be changed
     * @param newVerbose True is more logging through telemetry, false is the most basic amount
     * @throws InterruptedException
     * @deprecated The only reason I had this function was to make sure that the servos were properly initialised because
     * they could only be initialised after the match started or else the robot would be outside the 18 in cube. Now that
     * we have removed all the servos, there is no reason to initialise the robot like this.
     */
    @Deprecated
    public void initAndWait(DcMotor.RunMode newDefault, boolean newVerbose) throws InterruptedException
    {
        initialise(newDefault, newVerbose);
        waitForStart();
        Thread.sleep(100);
    }

    /**
     * To move the robot forward.
     *
     * @param speed    The speed the robot should go
     * @param distance The distance, or amount of motor encoder ticks, the robot should go.
     * @throws InterruptedException
     */
    public void move(double speed, double distance) throws InterruptedException
    {
        final int WAIT_BEFORE_MOVE = 100;

        if(speed > MAX_MOVE_SPEED)
        {
            telemetry.addData("Warning", "Move speed is larger than the max move speed");
            telemetry.update();
        }

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // This set of if statements just makes sure that if one parameter is negative
        // and the other is not then it turns the positive parameter to negative. This
        // makes sure that the robot actually moves in the direction that is inputed.
        if(Math.abs(speed) == -1*speed && Math.abs(distance) == distance) // If speed is negative and distance is positive
        {distance *= -1;}
        else if(Math.abs(distance) == distance*-1 && Math.abs(speed) == speed) // If distance is negative and speed is positive
        {speed *= -1;}

        // Instead of resetting the encoders, adding the distance to the current encoder
        // position for a quick and reliable way to set the target position for the drive
        // motors that move using encoders
        right1.setTargetPosition((int) (right1.getCurrentPosition() + distance));
        right3.setTargetPosition((int) (right3.getCurrentPosition() + distance));
        left1.setTargetPosition((int) (left1.getCurrentPosition() + distance));
        left3.setTargetPosition((int) (left3.getCurrentPosition() + distance));

        Thread.sleep(WAIT_BEFORE_MOVE); // Just to make sure that everything is set before the robot starts moving

        setDriveMotorSpeed(speed);

        // While the motors have not gotten to their target position keep moving
        while(opModeIsActive() &&
                right1.isBusy() &&
                right3.isBusy() &&
                left1.isBusy()  &&
                left3.isBusy()){
            Thread.sleep(50);

        }

        stopDriveMotors();

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION) // Set the drive motors back to the default runMode
        {
            resetDriveMotorMode();
        }

        Thread.sleep(END_WAIT);
    }

    /**
     * Moves the robot forward at the max move speed.
     *
     * @param distance The distance, or amount of motor encoder ticks, the robot should go.
     * @throws InterruptedException
     */
    public void move(double distance) throws InterruptedException
    {
        move(MAX_MOVE_SPEED, distance);
    }

    /**
     * To turn the robot.
     *
     * @param maxSpeed The max speed the robot will move at. The robot slows as it gets closer to the target deg for accuracy.
     * @param deg      The amount of degrees the robot should turn. Negative values turn right and positive turns left.
     * @throws InterruptedException
     */
    public void turn(double maxSpeed, double deg) throws InterruptedException
    {
        // The TURN_RANGE variable adjusts the range that the robot can stop in. While it would be nice to
        // have the robot stop on a dime it cant always and this makes the range one above and one below the
        // input variable when set to two, for example.
        final int TURN_RANGE = 2;
        final double MIN_SPEED = 0.05; // Exactly what it sounds like. At NSR it was 0.03
        final int WAIT_BEFORE_MOVE = 100;
        double targetHeading = gyro.getIntegratedZValue() + deg; // Resetting the gyro takes around 3 seconds and that's too long
        double speed = maxSpeed;

        if(maxSpeed > MAX_TURN_SPEED)
        {
            telemetry.addData("Warning", "Turning faster than max turn speed");
            telemetry.update();
        }


        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            Thread.sleep(WAIT_BEFORE_MOVE);
        }

        // While the gyro is not within the range (TURN_RANGE)
        while(!(gyro.getIntegratedZValue() <= targetHeading+(TURN_RANGE/2.0) && gyro.getIntegratedZValue() >= targetHeading-(TURN_RANGE/2.0)) && opModeIsActive())
        {
            speed = Range.clip(maxSpeed * Math.abs((gyro.getIntegratedZValue() - targetHeading) / deg), MIN_SPEED, maxSpeed);

            if(gyro.getIntegratedZValue() > targetHeading) // Right turn
            {
                setDriveMotorSpeed(speed, -1 * speed);
            }

            else //then its a left turn
            {
                setDriveMotorSpeed(-1 * speed, speed);
            }

            if(verbose)
            {
                gyroTelemetry(gyro);
                telemetry.addData("Robot Max speed: ", maxSpeed+"");
                telemetry.addData("Robot speed: ", speed+"");
                telemetry.update();
            }
        }

        stopDriveMotors();

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER) // last thing to do
        {
            resetDriveMotorMode();
        }

        if (verbose) {
            telemetry.addData("Done: ", "Turning. " + deg + " deg. Heading: " + gyro.getIntegratedZValue() + " Target Heading: " + targetHeading); telemetry.update();}

        Thread.sleep(END_WAIT);
    }

    /**
     * Turns the robot at the max turn speed
     *
     * @param deg The amount of degrees the robot should turn. Negative values turn right and positive turns left.
     * @throws InterruptedException
     */
    public void turn(double deg) throws InterruptedException
    {
        turn(MAX_TURN_SPEED, deg);
    }

    /**
     * Presses the beacon and if the beacon is the wrong color presses it again.
     *
     * @param speed       The speed the robot will move at. Recommended is 0.3
     * @param colorWanted This is whatever the color is wanted on the beacon after the function is over. Either Color.BLUE or Color.RED.
     * @throws InterruptedException
     */
    public void pressAndTest(double speed, int colorWanted) throws InterruptedException // recommended speed: 0.3
    {
        final double WAIT_BEFORE_MOVE = 5; // in sec // TODO: Test to see what the smallest value for this is
        final double MOVE_BACK_DIST = 1440*0.5 * -1;
        final double SECOND_PRESS_DIST = 1440 * 0.7 * -1;
        final int WAIT_BEFORE_READ = 500; // In ms
        final int SLOW_DIST = 9; // The distance from the wall to the robot where the robot will stop in cm
        final int STOP_DIST = 6;
        final double SLOW_SPEED = speed/2.0;

        frontColor.enableLed(false);

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Thread.sleep(100);
        }

        // move into beacon
        setDriveMotorSpeed(speed);

        while(frontRange.getDistance(DistanceUnit.CM) > SLOW_DIST)
        {
            if(verbose)
            {
                telemetry.addData("Speed", speed);
                rangeTelemety(frontRange);
                motorTelemetry(right1); // Just picked a random motor, can't have 'em all
                telemetry.update();
            }
        }

        setDriveMotorSpeed(SLOW_SPEED);

        while(frontRange.getDistance(DistanceUnit.CM) > STOP_DIST)
        {
            if(verbose)
            {
                telemetry.addData("Speed", SLOW_SPEED);
                rangeTelemety(frontRange);
                motorTelemetry(right1); // Just picked a random motor, can't have 'em all
                telemetry.update();
            }
        }

        stopDriveMotors();

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER) // last thing to do
        {
            resetDriveMotorMode();
        }

        double pressTime = getRuntime();

        sleep(WAIT_BEFORE_READ);

        if(frontColor.red() == 0 && frontColor.blue() == 0) // no need to test green because its useless :(
        {
            telemetry.addData("Warning", "Not close enough to sense beacon color");
            telemetry.update();
        }

        else if(frontColor.red() == 255 && frontColor.blue() == 255)
        {
            telemetry.addData("Error", "Color sensor is broken again");
            telemetry.update();
        }

        else if((frontColor.red() > frontColor.blue() && colorWanted == Color.BLUE) || (frontColor.blue() > frontColor.red() && colorWanted == Color.RED))
        {

            if(verbose)
            {
                colorTelemetry(frontColor);
                telemetry.update();
                frontColor.enableLed(true); // as a way to signel that it will go back in and press
            }

            move(speed, MOVE_BACK_DIST); // move back from wall

            while(getRuntime()-pressTime < WAIT_BEFORE_MOVE && opModeIsActive()){sleep(30);} // making sure to wait enough time before pressing again

            move(speed, -1*SECOND_PRESS_DIST); // move back in, hitting the button
        }
        else if(verbose) // if all else fails
        {
            colorTelemetry(frontColor);
            telemetry.update();
        }

        move(speed, MOVE_BACK_DIST); // move back from wall
    }

    /**
     * This function shoots and recocks the shooter. This function is nonblocking so for
     * accurate shooting wait around 300 ms after calling this function. If the touch
     * sensor is broken, this function will spin the cam until stopThread is true or the program is over.
     */
    public void shootThreaded() // TODO: Add a part to test if the button is broken by seeing if the method has been running for too long
    {
        if(!running)
        {
            new Thread(new Runnable() {
                public void run() {
                    running = true;

                    shooter.setPower(0.75);

                    while(touch.getState() && opModeIsActive() && !stopThread){}

                    while(!touch.getState() && opModeIsActive() && !stopThread){}

                    shooter.setPower(0);

                    running = false;
                    stopThread = false;
                }
            }).start();
        }
    }

    /**
     * Just shoots the shooter, does not recock and is blocking.
     *
     * @throws InterruptedException
     * @depricated because it does not recock and blocks other code. Use shootThreaded().
     */
    @Deprecated
    public void shoot() throws InterruptedException
    {
        shooter.setPower(0.9);

        Thread.sleep(500);

        shooter.setPower(0);
    }

    /**
     * This is a function to move the robot a certain distance and shoot at a certain point in that distance.
     *
     * @param speed           The speed the robot will move at
     * @param totalDist       The total distance, or motor encoder ticks, that the robot will move
     * @param distBeforeShoot The distance, or motor encoder ticks, that the robot will move before shooting
     * @throws InterruptedException
     */
    public void moveShootMove(double speed, double totalDist, double distBeforeShoot) throws InterruptedException
    {
        final int WAIT_TIME = 250;

        if(distBeforeShoot > totalDist)
        {
            telemetry.addData("Warning", "Please use moveShootMove correctly, function may act weird");
            telemetry.update();
        }

        if(distBeforeShoot != 0)
            move(speed, distBeforeShoot);

        shootThreaded();

        Thread.sleep(WAIT_TIME); // time for it to shoot

        if(distBeforeShoot < totalDist)
            move(speed, totalDist - distBeforeShoot);
    }

    /**
     * Function to set drivetrain motors to a different mode. right2 and left2 are set to RUN_WITHOUT_ENCODER
     * to prevent fuses from being blown.
     *
     * @param mode The mode that the drivetrain will be set to.
     */
    public void setDriveMotorMode(DcMotor.RunMode mode)
    {
        right1.setMode(mode);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Set to this so we don't blow fuses
        right3.setMode(mode);
        left1.setMode(mode);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Set to this so we don't blow fuses
        left3.setMode(mode);
    }

    /**
     * Resets the drivetrain motors to the default runMode
     */
    public void resetDriveMotorMode()
    {
        setDriveMotorMode(defualtRunMode);
    }

    /**
     * Resets the encoders on all drive motors, then the mode is set back to the default runMode
     *
     * @throws InterruptedException
     */
    public void resetEncoders() throws InterruptedException
    {
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.sleep(100);

        resetDriveMotorMode();
    }

    /**
     * Sets the speed of the motors on the drivetrain. The sides of the drivetrain are alternated so one side doesnt start before the other.
     *
     * @param leftSpeed The speed for the left side
     * @param rightSpeed The speed for the right side
     */
    public void setDriveMotorSpeed(double leftSpeed, double rightSpeed)
    {
        left1.setPower(leftSpeed);
        right1.setPower(rightSpeed);
        left2.setPower(leftSpeed);
        right2.setPower(rightSpeed);
        left3.setPower(leftSpeed);
        right3.setPower(rightSpeed);

    }

    /**
     * Sets the speed of the motors on the drivetrain
     *
     * @param speed The speed of the whole drivetrain (right and left sides)
     */
    public void setDriveMotorSpeed(double speed)
    {
        setDriveMotorSpeed(speed, speed);
    }

    /**
     * Sets the drive motors to 0 speed, stopping them. To make sure that the robot has come to a full
     * stop wait around 200 ms after calling this function.
     */
    public void stopDriveMotors()
    {
        setDriveMotorSpeed(0);
    }

    /**
     * Adds gyro sensor values to the telemetry but does not update the telemetry.
     *
     * @param sensorGyro which gyro the sensor values should be read from.
     */
    public void gyroTelemetry(ModernRoboticsI2cGyro sensorGyro)
    {
        telemetry.addData("Gyro: ", sensorGyro.getIntegratedZValue() + ""); // Has an infinite range (continuously adds when turning one way and subtracts when turning the other)
        telemetry.addData("Gyro Heading: ", sensorGyro.getHeading() + ""); // Has a range of -180 to 180 (wraps around, meaning that if it is at 180 and turns one degree, it wraps around)
    }

    /**
     * Adds range sensor values to the telemetry but does not update the telemetry.
     *
     * @param rangeSensor which range sensor values should be read from.
     */
    public void rangeTelemety(ModernRoboticsI2cRangeSensor rangeSensor)
    {
        telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM)); // what to use when trying to get values from a sensor
    }

    /**
     * Adds color sensor values to the telemetry but does not update the telemetry.
     *
     * @param color which color sensor values should be read from.
     */
    public void colorTelemetry(ColorSensor color)
    {
        telemetry.addData("Red", color.red());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Green", color.green());
        telemetry.addData("Alpha", color.alpha());
    }

    /**
     * Adds motor values to the telemetry but does not update.
     *
     * @param motor which motor values should be read from.
     */
    public void motorTelemetry(DcMotor motor)
    {
        telemetry.addData("Target Dist", motor.getTargetPosition());
        telemetry.addData("Target Dist", motor.getTargetPosition());
        telemetry.addData("Is Busy", motor.isBusy()?"Yes":"No");

    }
}
