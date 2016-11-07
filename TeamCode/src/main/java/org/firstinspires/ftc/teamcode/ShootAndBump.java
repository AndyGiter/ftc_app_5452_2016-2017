package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by mlowery2 on 11/7/2016.
 */
public class ShootAndBump extends LinearOpMode {

    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    Servo rightBp;
    Servo leftBp;
    Servo cannon;

    public void runOpMode() throws InterruptedException
    {
        initalize();
        waitForStart();

    }

    public void initalize()
    {
        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);

        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBp = hardwareMap.servo.get("right");
        leftBp  = hardwareMap.servo.get("left");
        cannon  = hardwareMap.servo.get("cannon");

        rightBp.scaleRange(0.1, 0.76);
        leftBp.scaleRange(0, 0.57);

        rightBp.setPosition(0.5);
        leftBp.setPosition(0.5);
        cannon.setPosition(0.7);
    }
}
