package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mlowery2 on 11/7/2016.
 */
@Autonomous(name = "Servo Testing", group ="testing")
public class servoTest extends LinearOpMode {

    Servo cannon;
    public void runOpMode() throws InterruptedException
    {

        cannon = hardwareMap.servo.get("cannon");
        cannon.setPosition(1);

        telemetry.addData("Current Position: ", cannon.getPosition());
        telemetry.update();

        waitForStart();

        cannon.setPosition(0);

        while(opModeIsActive())
        {
            telemetry.addData("Current Position: ", cannon.getPosition());
            telemetry.update();
        }
    }
}
