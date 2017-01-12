package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mlowery2 on 9/21/2016.
 */

/*
* Controls
*   Gamepad1:
*       Y: Toggle Slow Mode
*       Right Joystick: Right Motor
*       Left Joystick: Left Motors
*
*   Gamepdad2:
*       A: Spin the Snail Cam (One rotation is a shot)
*       Right Bumper: Collect inwards
*       Left Bumpers: Push out balls
* */

@TeleOp(name="Basic Teleop", group="Teleop")
public class BasicTeleop extends LinearBase {

    private final double SLOW_MOD = 0.3; // 30% of normal speed
    private boolean slow = false;
    private boolean press = false;

    public void runOpMode() throws InterruptedException
    {

        initalize(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while(opModeIsActive())
        {

            if(gamepad2.a) // TODO: Make a better shooting system that allows for one press shoot and reload and manual control
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

            if(press) // TODO: make a better system for this that deosn't prevent use of the joysticks
            {
                Thread.sleep(250);
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
