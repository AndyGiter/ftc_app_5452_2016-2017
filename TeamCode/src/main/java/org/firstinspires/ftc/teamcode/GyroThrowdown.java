package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


// delete ones that are not needed along with other crap that goes unused.
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by mlowery2 on 1/31/2017.
 *
 * This program is written to test the two gyros side by side IN AN ULTIMATE THROWDOWN
 * also maybe a turning optimization program?
 */
@Autonomous(name = "GYRO SHOWDOWN", group = "Sensor")
public class GyroThrowdown extends LinearBase {

    // The new, untested gyro sensor
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    public void runOpMode() throws InterruptedException
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode and do this at some point maybe
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        initalize(true);
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000); // Make sure this is in the right spot
        Thread.sleep(100);

        turnG(0.45, 90);

        while(opModeIsActive())
        {
            telemetry.addData("Gyro heading: ", gyro.getIntegratedZValue()+"");
            telemetry.addData("IMU heading: ", getStringHeading());
            telemetry.update();
        }

    }

    public void turnG(double maxSpeed, int deg) throws InterruptedException
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
                telemetry.addData("IMU heading: ", getStringHeading());
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

    // Make sure that negative is still right and if not change all the code i guess idk.
    public void turnIMU(double maxSpeed, int deg) throws InterruptedException
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
        while(!(getHeading() <= targetHeading+(TURN_RANGE/2.0) && getHeading() >= targetHeading-(TURN_RANGE/2.0)) && opModeIsActive())
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
                telemetry.addData("IMU heading: ", getStringHeading());
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

    private double getHeading() // Hopefully this works
    {
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    private String getStringHeading()
    {
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
