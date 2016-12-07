package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mlowery2 on 9/21/2016.
 */

/*
* TODO:
* Clean up program
* Slow mode
* Swap motor direction button(s)
**/

/*
* Controls
* Right Joystick: Right Motors
* Left Joystick: Left Motors
* A: Old ball shooter
*
* TODO:
* RT: Ball collector
* A: Ball Shooter
* DPad up: Forward Mode
* DPad down: Backward Mode
* DPad left: Slow
* DPad right: Fast
* */

@TeleOp(name="Basic Teleop", group="Teleop")

public class BasicTeleop extends LinearBase {

    private final double SLOW_MOD = 0.3; // 30% of normal speed
    private boolean slow = false;
    private Direction currentDirection = Direction.FORWARD;

    public void runOpMode() throws InterruptedException
    {

        initalize(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a) // TODO: Update when new ball shooter is on the robot
            {
                cannon.setPosition(0);
            }

            if(gamepad1.dpad_left && !slow)
            {
                slow = true;
            }
            else if(gamepad1.dpad_right && slow)
            {
                slow = false;
            }

            if(gamepad1.dpad_up && currentDirection == Direction.BACKWARD)
            {
                currentDirection = Direction.FORWARD;
            }
            else if(gamepad1.dpad_down && currentDirection == Direction.FORWARD)
            {
                currentDirection = Direction.BACKWARD;
            }

            left1.setPower(-1 * gamepad1.right_stick_y);
            left2.setPower(-1 * gamepad1.right_stick_y);
            right1.setPower(-1 * gamepad1.left_stick_y);
            right2.setPower(-1 * gamepad1.left_stick_y);

        }
    }
}
