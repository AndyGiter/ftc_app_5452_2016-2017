package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 9/21/2016.
 *
 * This is the tele op program for team 5452
 *
 * TODO: Add control comment (maybe wait until the end of the year)
 */

@TeleOp(name="New Teleop", group="Teleop")
public class BasicTeleop extends LinearBase { // TODO: Look into why the usb hub didn't work

    private final double SLOW_MOD = 0.3; // 30% of normal speed TODO: Retest for new drive train and maybe new controls
    private boolean slow = false;

    private boolean press = false;
    private final int PRESS_TIME = 250;

    public void runOpMode() throws InterruptedException
    {

        initalize(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        Thread.sleep(100);

        while(opModeIsActive())
        {
            if(gamepad1.start || gamepad2.start) // an emergency button to on the fly add logging
            {
                verbose = !verbose;
                press = true;
            }

            if(gamepad1.a)
            {
                shootThreaded();
            }

            if(gamepad1.right_bumper)
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
                telemetry.addData("Shooter is busy?", shooter.isBusy());
                telemetry.addData("Current Shooter Pos", shooter.getCurrentPosition());
                telemetry.addData("Shooter Target Pos", shooter.getTargetPosition());
                telemetry.addData("A button", gamepad1.a);
                telemetry.addData("Button State", touch.getState());
                telemetry.addData("Running", running);
            }
            telemetry.update();

            if(press) // TODO: make a better system for this that doesn't prevent use of the joysticks. Threading?
            {
                Thread.sleep(PRESS_TIME);
                press = false;
            }
        }
    }
}
