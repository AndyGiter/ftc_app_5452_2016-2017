package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mlowery2 on 9/21/2016.
 */


@TeleOp(name="Basic Teleop", group="Testing")

public class BasicTeleop extends LinearOpMode {

    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    Servo rightBp;
    Servo leftBp;

    final float DEADZONE = 0.200f;
    double pos = 0.5;

    public void runOpMode() throws InterruptedException {

        left1  = hardwareMap.dcMotor.get("left1");
        left2  = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        left1.setDirection(DcMotor.Direction.REVERSE); // make sure this works
        left2.setDirection(DcMotor.Direction.REVERSE);

        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // see if this works the other way
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBp = hardwareMap.servo.get("right");
        leftBp  = hardwareMap.servo.get("left");

        gamepad1.setJoystickDeadzone(DEADZONE);
        gamepad2.setJoystickDeadzone(DEADZONE);

        rightBp.scaleRange(0.1, 0.76);
        leftBp.scaleRange(0, 0.57);

        rightBp.setPosition(0.5);
        leftBp.setPosition(0.5);


        telemetry.addData("Revision", "3, now with servos");
        telemetry.update();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.x) // left
            {
                leftBp.setPosition(1);
                rightBp.setPosition(1);
            }
            else if(gamepad1.y) // middle
            {
                leftBp.setPosition(0.5);
                rightBp.setPosition(0.5);
            }
            else if(gamepad1.b) // right
            {
                leftBp.setPosition(0);
                rightBp.setPosition(0);
            }


            moveRight(-1 * gamepad1.right_stick_y, -1);
            moveLeft(-1 * gamepad1.left_stick_y, -1);

        }
    } // End of running code
     //  Start of Functions

    public void moveLeft(float speed, int dist)
    {
        if(dist<0) // so that I dont have to input a distance for the encodes to go
        {
            left1.setPower(speed);
            left2.setPower(speed);
        }
    }

    public void moveRight(float speed, int dist)
    {
        if(dist<0)
        {
            right1.setPower(speed);
            right2.setPower(speed);
        }
    }

    public void moveForward(float speed, int dist) // not for use in this code
    {
        if(dist<0)
        {
            right1.setPower(speed);
            right2.setPower(speed);
            left1.setPower(speed);
            left2.setPower(speed);
        }
    }
}
