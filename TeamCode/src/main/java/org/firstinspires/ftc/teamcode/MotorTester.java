package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mlowery2 on 1/10/2017.
 */

@TeleOp(name="Motor Testing", group="testing")
public class MotorTester extends LinearBase {

    public void runOpMode() throws InterruptedException
    {
        initalize(true);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        Thread.sleep(100);

        while(opModeIsActive())
        {
            if(gamepad1.dpad_left)
            {
                shooter.setPower(0.3);
            }
            else if(gamepad1.dpad_right)
            {
                shooter.setPower(-0.3);
            }
            else
            {
                shooter.setPower(0);
            }

            if(gamepad1.a)
            {
                shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Thread.sleep(250);
                shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("Current Encoder Pos", shooter.getCurrentPosition());
            telemetry.update();
        }
    }
}
