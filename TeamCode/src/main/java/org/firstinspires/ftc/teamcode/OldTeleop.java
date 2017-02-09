package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mlowery2 on 9/21/2016.
 *
 * This is just a program to use if the touch sensor is broken
 */

@TeleOp(name="Old Teleop", group="Teleop")
public class OldTeleop extends LinearBase {

    private final double SLOW_MOD = 0.3; // 30% of normal speed
    private boolean slow = false;

    private boolean press = false;

    private DcMotor.RunMode shootMode = DcMotor.RunMode.RUN_USING_ENCODER;

    public void runOpMode() throws InterruptedException
    {

        initalize(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setMode(shootMode);
        waitForStart();
        Thread.sleep(100);

        while(opModeIsActive())
        {
            if(gamepad1.start || gamepad2.start) // an emergency button to on the fly add logging
            {
                verbose = !verbose;
                press = true;
            }

            if(shooter.getCurrentPosition() >= shooter.getTargetPosition()) // the inequality is negative because the motor goes backwards
             {
                if(gamepad1.a)
                {
                    if(shootMode != DcMotor.RunMode.RUN_TO_POSITION)
                    {
                        shootMode = DcMotor.RunMode.RUN_TO_POSITION;
                        shooter.setMode(shootMode);
                    }

                    shooter.setTargetPosition(shooter.getCurrentPosition() + (3390));
                    shooter.setPower(0.9);
                }

                else if(gamepad1.b)
                {
                    if(shootMode != DcMotor.RunMode.RUN_USING_ENCODER)
                    {
                        shootMode = DcMotor.RunMode.RUN_USING_ENCODER;
                        shooter.setMode(shootMode);
                    }

                    shooter.setPower(0.9);
                }

                else
                {
                    shooter.setPower(0);
                }
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

            if(press) // TODO: make a better system for this that deosn't prevent use of the joysticks Threading?
            {
                Thread.sleep(250);
                press = false;
            }
        }
    }
}
