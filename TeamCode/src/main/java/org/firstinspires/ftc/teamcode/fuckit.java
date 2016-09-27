package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

/**
 * Created by mlowery2 on 9/27/2016.
 */
@TeleOp(name="work pls", group="Testing")
public class fuckit extends LinearOpMode { // Just to find the min max positions of any servo
    Servo test;
    double pos = 0.5;

    public void runOpMode() throws InterruptedException
    {
        test = hardwareMap.servo.get("left");

        waitForStart();

        for(double i = 0; i<=1; i+=0.1)
        {
            test.setPosition(i);
            telemetry.addData("", i);
            telemetry.update();
            Thread.sleep(500);
        }

        for(double i = 1; i>=0; i-=0.1)
        {
            test.setPosition(i);
            telemetry.addData("", i);
            telemetry.update();
            Thread.sleep(500);
        }
    }
}
