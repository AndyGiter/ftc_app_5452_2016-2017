package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 11/22/2016.
 */

//TODO: finish this shit
@TeleOp(name = "Sensor tester", group = "diagnostic")
public class SensorTester extends LinearBase {

    private boolean ledState = true;

    public void runOpMode() throws InterruptedException
    {
        initalize(true);
        front.enableLed(true);
        waitForStart();
        Thread.sleep(100);

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                ledState = !ledState;
                front.enableLed(ledState);
                Thread.sleep(250);
            }

            telemetry.addData("LED State", ledState);
            colorTelemetry(front, hsvValuesFront);
            telemetry.update();
            Thread.sleep(50);
        }
    }

}
