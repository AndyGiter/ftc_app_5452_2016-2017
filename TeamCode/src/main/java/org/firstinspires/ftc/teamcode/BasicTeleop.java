package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 9/21/2016.
 *
 * This is the tele op program for team 5452
 *
 * Gamepad1:
 *  Start: Add additional logging
 *  A: Automatic shoot and reload
 *  B: Manual turn of the snail cam
 *  Right Bumper: Collect balls
 *  Left Bumper: Push out balls
 *  Right Trigger: Super slow mode (30% Speed)
 *  Left Trigger: Slow Mode (50% Speed)
 *  Y: Toggle default speed
 *  Right Stick (up and down): Right motors forwards and backwards
 *  Left Stick (up and down): Left Motors forwards and backwards
 */

/*
* Two Controller scheme (if implemented)
* Move collecting and shooting to gamepad2
* Change slowmode to bumpers
* */

@TeleOp(name="New Teleop", group="Teleop")
public class BasicTeleop extends LinearBase {

    private double speedMod = 1;
    private boolean defaultSpeed = false; // true is to use MAX_SPEED, false is
    private final double MAX_SPEED = 1;
    private final double SUB_MAX_SPEED = 0.75;
    private final double SLOW_MOD = 0.5; // 50% of normal speed
    private final double SUPER_SLOW_MOD = 0.3;

    private boolean press = false;
    private final int PRESS_TIME = 250;

    private final double TRIGGER_THRESHOLD = 0.35; // Below this value, the trigger does not count as being pressed

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
            else if(gamepad1.b && !running)
            {
                shooter.setPower(0.9);
            }
            else if(!running)
            {
                shooter.setPower(0);
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
                defaultSpeed = !defaultSpeed;
                press = true;
            }

            if(gamepad1.right_trigger > TRIGGER_THRESHOLD)
            {
                speedMod = SUPER_SLOW_MOD;
            }

            else if(gamepad1.left_trigger > TRIGGER_THRESHOLD)
            {
                speedMod = SLOW_MOD;
            }
            else
            {
                speedMod = defaultSpeed?MAX_SPEED:SUB_MAX_SPEED;
            }

            right1.setPower(-1 * gamepad1.right_stick_y * speedMod);
            right2.setPower(-1 * gamepad1.right_stick_y * speedMod);
            right3.setPower(-1 * gamepad1.right_stick_y * speedMod);
            left1.setPower(-1 * gamepad1.left_stick_y * speedMod);
            left2.setPower(-1 * gamepad1.left_stick_y * speedMod);
            left3.setPower(-1 * gamepad1.left_stick_y * speedMod);

            telemetry.addData("Current Speed Mod", (speedMod*100)+"%");
            if(verbose)
            {
                telemetry.addData("A button pressed?", gamepad1.a?"Yes" : "No");
                telemetry.addData("Touch Sensor pressed?", touch.getState()?"Yes":"No");
                telemetry.addData("Shooter Spin Thread Running", running?"Yes":"No");
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
