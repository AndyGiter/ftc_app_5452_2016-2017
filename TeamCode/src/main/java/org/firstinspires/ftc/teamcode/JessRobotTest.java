package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by FTC on 10/18/2016.
 */
@Autonomous(name = "first autonomous", group = "Testing")
public class JessRobotTest extends LinearOpMode {
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    public void runOpMode() throws InterruptedException {
        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        left1.setPower(0.75);
        right1.setPower(0.75);
        left2.setPower(0.75);
        right2.setPower(0.75);
        Thread.sleep(1000);
        left1.setPower(0.75);
        right1.setPower(0.50);
        left2.setPower(0.75);
        right2.setPower(0.50);

        Thread.sleep(1000);



    }

}