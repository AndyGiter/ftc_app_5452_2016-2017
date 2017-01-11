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
* A: Shoot 1 ball (2 motor rotations)
* Right Trigger: Collect (not toggled)
* Left Trigger: Collect Backwards
* Y: Drive train Slow Mode
* Dpad Right: Lower ball holder
* Dpad LeftL: Raise ball holder
*
* */

@TeleOp(name="Basic Teleop", group="Teleop")

public class BasicTeleop extends LinearBase {

    private final double SLOW_MOD = 0.3; // 30% of normal speed
    private boolean slow = false;
    private boolean press = false;
    private int newPressTime = 0;

    public void runOpMode() throws InterruptedException
    {

        initalize(DcMotor.RunMode.RUN_USING_ENCODER, true);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(opModeIsActive())
        {
            /*
             if(gamepad1.b)
            {
                shooter.setPower(-0.75);
            }

            if(shooter.getCurrentPosition() <= shooter.getTargetPosition()) // make a better system to control the shooter
            {
                if(gamepad1.a)
                {
                    //shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION); // at some point test how much lag this creates

                    shooter.setTargetPosition(shooter.getCurrentPosition() + (-3395)); // make sure this spins the right way
                    shooter.setPower(0.9);
                }

                else
                {
                    shooter.setPower(0);
                }
            }
            */

            if(gamepad2.a)
            {
                shooter.setPower(-0.9);
            }
            else
            {
                shooter.setPower(0);
            }


            if(gamepad2.right_bumper)
            {
                collector.setPower(-0.9);
            }
            else if(gamepad2.left_bumper)
            {
                collector.setPower(0.9);
            }
            else
            {
                collector.setPower(0);
            }

            if(gamepad1.y)
            {
                slow = !slow;

                press = true;
            }

            right1.setPower(-1 * gamepad1.right_stick_y * (slow?SLOW_MOD:1));
            right2.setPower(-1 * gamepad1.right_stick_y * (slow?SLOW_MOD:1));
            left1.setPower(-1 * gamepad1.left_stick_y * (slow?SLOW_MOD:1));
            left2.setPower(-1 * gamepad1.left_stick_y * (slow?SLOW_MOD:1));

            if(press) // make a better system for this that deosn't prevent use of the joysticks
            {
                Thread.sleep(200);
                press = false;
            }

            if(verbose)
            {
                telemetry.addData("Shooter is busy?", shooter.isBusy());
                telemetry.addData("Current Shooter Pos", shooter.getCurrentPosition());
                telemetry.addData("Shooter Target Pos", shooter.getTargetPosition());
            }

            telemetry.addData("Is slow mode on", slow);
            telemetry.update();
        }
    }
}
