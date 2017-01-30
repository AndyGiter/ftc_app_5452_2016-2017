package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
*       Start: Toggle Logging
*
*   Gamepdad2:
*       A: Spin the Snail Cam (One rotation is a shot)
*       Right Bumper: Collect inwards
*       Left Bumpers: Push out balls
*       Start: Toggle Logging
* */

@TeleOp(name="Basic Teleop", group="Teleop")
public class BasicTeleop extends LinearBase { // TODO: Look into why the usb hub didn't work

    private final double SLOW_MOD = 0.3; // 30% of normal speed
    private boolean slow = false;

    private boolean press = false;

    private boolean reset = false; // for if the shooter motor is currently moving to be reset
    private boolean ignoreTouch = false; // for if the touch sensor should be ignored
    private boolean initPos; // This is for if the the snail cam doesnt start in the init position

    private DcMotor.RunMode shootMode = DcMotor.RunMode.RUN_USING_ENCODER;

    public void runOpMode() throws InterruptedException
    {

        initalize(DcMotor.RunMode.RUN_USING_ENCODER); //TODO: Test with DcMotor.RunMode.RUN_WITHOUT_ENCODER
        shooter.setMode(shootMode);
        initPos = touch.getState();
        waitForStart();
        Thread.sleep(100);

        while(opModeIsActive())
        {
            if(gamepad1.start || gamepad2.start) // an emergency button to on the fly add logging
            {
                verbose = !verbose;
                press = true;
            }

            // TODO: Maybe make a way to switch back to this incase the button breaks or something, currently broken since motor is reversed
            /*
            if(shooter.getCurrentPosition() <= shooter.getTargetPosition()) // the inequality is negative because the motor goes backwards
             {
                if(gamepad1.a)
                {
                    if(shootMode != DcMotor.RunMode.RUN_TO_POSITION)
                    {
                        shootMode = DcMotor.RunMode.RUN_TO_POSITION;
                        shooter.setMode(shootMode);
                    }

                    shooter.setTargetPosition(shooter.getCurrentPosition() + (-3390));
                    shooter.setPower(-0.9);
                }

                else if(gamepad1.b)
                {
                    if(shootMode != DcMotor.RunMode.RUN_USING_ENCODER)
                    {
                        shootMode = DcMotor.RunMode.RUN_USING_ENCODER;
                        shooter.setMode(shootMode);
                    }

                    shooter.setPower(-0.9);
                }

                else
                {
                    shooter.setPower(0);
                }
            }
            */



            if(!initPos && (gamepad1.a || gamepad1.b)) // To reset the shooter to the starting position
            {
                shooter.setPower(0.9);
                initPos = touch.getState();
            }

            else if(!initPos)
            {
                shooter.setPower(0);
                initPos = touch.getState();
            }

            else
            {
                if (ignoreTouch && reset && !touch.getState()) // TODO: Make sure these are in the best order
                {
                    ignoreTouch = false;
                }

                else if (!ignoreTouch && reset && touch.getState())
                {
                    reset = false;
                    shooter.setPower(0);
                }

                else if (touch.getState() && gamepad1.a && !reset && !ignoreTouch)
                {
                    ignoreTouch = true;
                    reset = true;

                    shooter.setPower(0.8);
                }
            }

            if(gamepad1.right_bumper) // TODO: Look into triggers
            {
                collector.setPower(-0.9);
            }
            else if(gamepad1.left_bumper)
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

            telemetry.addData("Is slow mode on", slow);
            if(verbose)
            {
                telemetry.addData("InitPos", initPos);
                telemetry.addData("IgnoreTouch", ignoreTouch);
                telemetry.addData("Reset", reset);
                telemetry.addData("A button", gamepad1.a);
                telemetry.addData("Button State", touch.getState());
            }
            telemetry.update();

            if(press) // TODO: make a better system for this that deosn't prevent use of the joysticks
            {
                Thread.sleep(250);
                press = false;
            }
        }
    }
}
