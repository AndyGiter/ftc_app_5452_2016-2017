package org.firstinspires.ftc.teamcode;

// Basic Needed stuff (some needed in the program that this will implement.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Sensors
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

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
    DcMotor left3;
    DcMotor right1;
    DcMotor right2;
    DcMotor right3;

    DcMotor shooter;
    DcMotor collector;

    Servo sonarServo;
    Servo pressServo;
    //Servo funnelLeft;
    //Servo funnelRight;

    ColorSensor frontColor;

    private I2cAddr i2cAddrFrontColor = I2cAddr.create8bit(0x4c);

    DigitalChannel touch;

    ModernRoboticsI2cGyro gyro; // Default I2C address 0x20

    ModernRoboticsI2cRangeSensor frontRange; // The range sensor on the front of the robot
    ModernRoboticsI2cRangeSensor sonarRange; // The range sensor under the side button presser
    ModernRoboticsI2cRangeSensor complementaryRange; // The range sensor thats also on the side with pressRange, but not under the side button pressor

    private I2cAddr i2cAddrFrontRange = I2cAddr.create8bit(0x28); // default I2C address for the range sensor
    private I2cAddr i2cAddrSonarRange = I2cAddr.create8bit(0x4c);
    private I2cAddr i2cAddrConplementaryRange = I2cAddr.create8bit(0x4c);

    OpticalDistanceSensor lightBottom;

    DcMotor.RunMode defualtRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    private final float DEADZONE = 0.200f;

    boolean verbose = false;

    final double MAX_MOVE_SPEED = 0.85; // YEAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHH
    final double MAX_TURN_SPEED = 0.3; // You spin me right round baby, right round

    final int END_WAIT = 200;

    public boolean running = false; // for if the shoot thread is running

    public void initalize() throws InterruptedException
    {
        double start = getRuntime();

        telemetry.addData("Done: ", "Starting init"); telemetry.update();

        // Gyro (this comes first so we can do other things, like initalizing other things, while this calibrates.)
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
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
        left2.setDirection(DcMotor.Direction.REVERSE); // (Crazy Train) https://www.youtube.com/watch?v=RMR5zf1J1Hs

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Resets the encoders on the motors

        Thread.sleep(150);

        resetDriveMotorMode(); // sets all the motors to the default runMode

        sonarServo = hardwareMap.servo.get("sonar");
        pressServo = hardwareMap.servo.get("press");

        collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        //Color Sensor Setup
        frontColor = hardwareMap.colorSensor.get("front");

        frontColor.setI2cAddress(i2cAddrFrontColor);

        frontRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "frontRange");
        sonarRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sonarRange");
        complementaryRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "complementaryRange");

        frontRange.setI2cAddress(i2cAddrFrontRange);
        sonarRange.setI2cAddress(i2cAddrSonarRange);
        complementaryRange.setI2cAddress(i2cAddrConplementaryRange);

        lightBottom = hardwareMap.opticalDistanceSensor.get("light");

        // Setting the deadzone for the gamepads
        gamepad1.setJoystickDeadzone(DEADZONE);
        gamepad2.setJoystickDeadzone(DEADZONE);

        if(verbose){telemetry.addData("Done: ","Everything but gyro. Took " + (getRuntime()-start) + " seconds"); telemetry.update();}

        while (!isStopRequested() && gyro.isCalibrating())  { // Make sure that the gyro is calibrated
            Thread.sleep(50);
        }


        if(verbose){telemetry.addData("Done: ","Initalizing. Took " + (getRuntime()-start) + " seconds"); telemetry.update();}
        else{telemetry.addData("Done", "Initalizing"); telemetry.update();}

        frontColor.enableLed(false);
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

        //This if tests if the speed is negative and sets the distance to be negative.
        //I have no idea if this is needed when moving backwards
        //TODO: figure that out
        if(Math.abs(speed) == -1*speed && Math.abs(distance) == distance) // If speed is negative and distance is positive
        {distance *= -1;}
        else if(Math.abs(distance) == distance*-1 && Math.abs(speed) == speed) // If distance is negative and speed is positive
        {speed *= -1;}

        right1.setTargetPosition((int) (right1.getCurrentPosition() + distance));
        right2.setTargetPosition((int) (right2.getCurrentPosition() + distance));
        right3.setTargetPosition((int) (right3.getCurrentPosition() + distance));
        left1.setTargetPosition((int) (left1.getCurrentPosition() + distance));
        left2.setTargetPosition((int) (left2.getCurrentPosition() + distance));
        left3.setTargetPosition((int) (left3.getCurrentPosition() + distance));

        Thread.sleep(WAIT_BEFORE_MOVE);

        right1.setPower(speed);
        left1.setPower(speed);
        right2.setPower(speed);
        left2.setPower(speed);
        right3.setPower(speed);
        left3.setPower(speed);

        while(opModeIsActive() && right1.isBusy() &&
                right2.isBusy() &&
                right3.isBusy() &&
                left1.isBusy()  &&
                left2.isBusy()  &&
                left3.isBusy()){
            Thread.sleep(50);

        }

        right1.setPower(0);
        left1.setPower(0);
        right2.setPower(0);
        left2.setPower(0);
        right3.setPower(0);
        left3.setPower(0);

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION) // last thing to do
        {
            resetDriveMotorMode();
        }

        Thread.sleep(END_WAIT);
    }

    /*
     * Negative deg value turns right
     * Positive is left
     */
    public void turn(double maxSpeed, double deg) throws InterruptedException
    {
        final int TURN_RANGE = 2;
        final double MIN_SPEED = 0.03; // play around with this
        final int WAIT_BEFORE_MOVE = 100;
        double targetHeading = gyro.getIntegratedZValue() + deg;
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
                right1.setPower(-1 * speed);
                left1.setPower(speed);
                right2.setPower(-1 * speed);
                left2.setPower(speed);
                right3.setPower(-1 * speed);
                left3.setPower(speed);
            }

            else //then its a left turn
            {

                left1.setPower(-1 * speed);
                right1.setPower(speed);
                left2.setPower(-1 * speed);
                right2.setPower(speed);
                left3.setPower(-1 * speed);
                right3.setPower(speed);
            }

            if(verbose)
            {
                gyroTelemetry(gyro);
                telemetry.addData("Robot Max speed: ", maxSpeed+"");
                telemetry.addData("Robot speed: ", speed+"");
                telemetry.update();
            }
        }

        right1.setPower(0);
        left1.setPower(0);
        right2.setPower(0);
        left2.setPower(0);
        right3.setPower(0);
        left3.setPower(0);

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER) // last thing to do
        {
            resetDriveMotorMode();
        }


        if (verbose) {
            telemetry.addData("Done: ", "Turning. " + deg + " deg. Heading: " + gyro.getIntegratedZValue() + " Target Heading: " + targetHeading); telemetry.update();}

        Thread.sleep(END_WAIT);
    }

    public void pressAndTest(double speed, int colorWanted) throws InterruptedException // recomended speed: 0.3
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
        }

        Thread.sleep(100);

        // move into beacon
        right1.setPower(speed);
        left1.setPower(speed);
        right2.setPower(speed);
        left2.setPower(speed);
        right3.setPower(speed);
        left3.setPower(speed);

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

        right1.setPower(SLOW_SPEED);
        left1.setPower(SLOW_SPEED);
        right2.setPower(SLOW_SPEED);
        left2.setPower(SLOW_SPEED);
        right3.setPower(SLOW_SPEED);
        left3.setPower(SLOW_SPEED);

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

        right1.setPower(0);
        left1.setPower(0);
        right2.setPower(0);
        left2.setPower(0);
        right3.setPower(0);
        left3.setPower(0);

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

    public void shootThreaded()
    {
        if(!running)
        {
            new Thread(new Runnable() {
                public void run() {
                    running = true;

                    shooter.setPower(0.9);

                    while(touch.getState() && opModeIsActive()){}

                    while(!touch.getState() && opModeIsActive()){}

                    shooter.setPower(0);

                    running = false;
                }
            }).start();
        }
    }

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

    // Argument speed also determines direction. Positive moves forward, negative is backward. The ball sweeper is the front
    public void moveParallelUntilLine(double speed)
    {
        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if(defualtRunMode != DcMotor.RunMode.RUN_USING_ENCODER)
        {
            resetDriveMotorMode();
        }

    }

    public void pressColor(int colorWanted)
    {
        final double SPEED = 0.3; // This is going to be a constant so that all instances of this function run the same

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(defualtRunMode != DcMotor.RunMode.RUN_TO_POSITION)
        {
            resetDriveMotorMode();
        }
    }

    //Made so that you can use this as the degree argument to a turning function and have the angle of the robot be corrected to be parallel to the wall
    public double angleDifFromWall()
    {
        final int TEST_POS_COUNT = 5; // testing 5 different positions
        final int FOV = 90; // The amount of degrees the servo could spin to see (make sure to test this)
        double[][] values = new double[TEST_POS_COUNT][2]; // [x][0] is the servo position, and [x][1] is the value there

        return 1; // Make sure to actually return something at the end

    }

    public void setDriveMotorMode(DcMotor.RunMode mode)
    {
        right1.setMode(mode);
        right2.setMode(mode);
        right3.setMode(mode);
        left1.setMode(mode);
        left2.setMode(mode);
        left3.setMode(mode);
    }

    public void resetDriveMotorMode()
    {
        setDriveMotorMode(defualtRunMode);
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

    public void colorTelemetry(ColorSensor color)
    {
        telemetry.addData("Red", color.red());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Green", color.green());
        telemetry.addData("Alpha", color.alpha());
    }

    public void motorTelemetry(DcMotor motor)
    {
        telemetry.addData("Target Dist", motor.getTargetPosition());
        telemetry.addData("Target Dist", motor.getTargetPosition());
        telemetry.addData("Is Busy", motor.isBusy()?"Yes":"No");

    }

    public void lightTelemetry(OpticalDistanceSensor odsSensor)
    {
        telemetry.addData("Raw",    odsSensor.getRawLightDetected());
        telemetry.addData("Normal", odsSensor.getLightDetected());
    }
}
