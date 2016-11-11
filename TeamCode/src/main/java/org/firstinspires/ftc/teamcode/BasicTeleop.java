package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mlowery2 on 9/21/2016.
 */


@TeleOp(name="Basic Teleop", group="Teleop")

public class BasicTeleop extends LinearBase {

    public void runOpMode() throws InterruptedException {

        initalize();
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
                leftBp.setPosition(RBP_INIT);
                rightBp.setPosition(LBP_INIT);
            }
            else if(gamepad1.b) // right
            {
                leftBp.setPosition(0);
                rightBp.setPosition(0);
            }
            else if(gamepad1.a)
            {
                cannon.setPosition(0);
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
}
