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
@TeleOp(name="Servo Teleop", group="Testing")
public class ServoTeleop extends LinearOpMode { // Just to find the min max positions of any servo
    Servo test;
    double pos = 0.5;

    public void runOpMode() throws InterruptedException
    {
        test = hardwareMap.servo.get("right");

        telemetry.addData("Revision", "4");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.x && pos<1)
            {
                pos += 0.01;
            }
            else if(gamepad1.b && pos>0)
            {
                pos -= 0.01;
            }

            telemetry.addData("Servo Position", "%5.2f", pos);
            telemetry.update();
            test.setPosition(pos);
            Thread.sleep(100);
        }
    }
}
