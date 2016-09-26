package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by mlowery2 on 9/21/2016.
 */

@Autonomous(name = "Testing Opmode", group = "Testing")

public class BasicTeleop extends LinearOpMode {

    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    public void runOpMode() throws InterruptedException {

        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");


    }

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
