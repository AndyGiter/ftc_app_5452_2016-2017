package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 11/22/2016.
 */

//TODO: finish this shit
@Autonomous(name = "Sensor tester", group = "diagnostic")
public class SensorTester extends LinearBase {

    int sensorNum = 0;

    public void runOpMode() throws InterruptedException
    {
        initalize(true);

        red.enableLed(true);
        blue.enableLed(true);
        waitForStart();

        while(opModeIsActive()) {
            updateHsv();
            colorTelemetry(red, hsvValuesRed[0]);
            colorTelemetry(blue, hsvValuesBlue[0]);
            telemetry.update();

            Thread.sleep(30);
        }
    }

    public void changeLightOn()
    {
        red        .enableLed(false);
        blue       .enableLed(false);
        bottomLeft .enableLed(false);
        bottomRight.enableLed(false);

        if(sensorNum < 4)
            sensorNum += 1;
        else
            sensorNum = 1;

        switch(sensorNum)
        {
            case 1:
                red.enableLed(true);
                break;
            case 2:
                blue.enableLed(true);
                break;
            case 3:
                bottomLeft.enableLed(true);
                break;
            case 4:
                bottomLeft.enableLed(true);
                break;

        }
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

    public void colorTelemetry(ColorSensor color, float hsvVals)
    {
        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Green: ", color.green());
        telemetry.addData("HSV", hsvVals);
    }
}
