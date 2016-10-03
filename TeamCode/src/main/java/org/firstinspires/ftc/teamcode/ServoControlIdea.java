package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by mlowery2 on 9/27/2016.
 */
@TeleOp(name="Servo Control Proof of Concept", group="Testing")
public class ServoControlIdea extends LinearOpMode { // Just to find the min max positions of any servo
    Servo test;
    double pos = 0.5;

    public void runOpMode() throws InterruptedException
    {
        test = hardwareMap.servo.get("right");
        test.scaleRange(0.1, 0.76);
        test.setPosition(0.5);

        boolean pos = false;

        telemetry.addData("Revision", "7");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                pos = !pos;
                test.setPosition(pos?0:1);
                Thread.sleep(250);
            }
        }
    }
}
