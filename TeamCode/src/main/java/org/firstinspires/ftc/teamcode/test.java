package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by mlowery2 on 9/21/2016.
 */

@Autonomous(name = "Testing Opmode", group = "Testing")

public class test extends LinearOpMode {

    DcMotor left;
    DcMotor right;

    public void runOpMode() throws InterruptedException
    {
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        waitForStart();
        left.setPower(0.75);
        right.setPower(0.75);
        Thread.sleep(1000);
        left.setPower(0.75);
        right.setPower(0.50);

        Thread.sleep(1000);
    }
}
