package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by mlowery2 on 9/27/2016.
 */
@TeleOp(name="Servo Teleop", group="Testing")
public class ServoTeleop extends LinearBase { // Just to find the min max positions of any servo
    double pos = 0;
    final double changeBy = 0.01;
    final long waitTime = 100;

    boolean ledState = true;

    public void runOpMode() throws InterruptedException
    {
        initalize(true);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.x && pos<1)
            {
                pos += changeBy;
            }
            else if(gamepad1.b && pos>0)
            {
                pos -= changeBy;
            }

            updateHsv();

            telemetry.addData("Servo Position ", "%5.2f", pos);
            telemetry.update();
            cannon.setPosition(pos);
            Thread.sleep(waitTime);
        }
    }
}
