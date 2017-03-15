package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by mlowery2 on 3/15/2017.
 */

@TeleOp(name = "Servo Teleop", group = "Testing")
public class ServoTeleop extends LinearBase {

    Servo servo;
    final double CHANGE_RATE = 0.05; // Simulates CHANGE_RATE in sonar functions
    final int WAIT_AFTER_PRESS = 100;// Simulates WAIT_BEFORE_READ in sonar functions
    boolean press = false;

    public void runOpMode() throws InterruptedException {
        initalize(true);
        servo = sonarServo; // where to change what servo is moved
        waitForStart();
        Thread.sleep(100);

        while (opModeIsActive())
        {
            if (gamepad1.dpad_left)
            {
                servo.setPosition(Range.clip(servo.getPosition() + CHANGE_RATE, 0, 1));
                press = true;
            }
            else if (gamepad1.dpad_right)
            {
                servo.setPosition(Range.clip(servo.getPosition() - CHANGE_RATE, 0, 1));
                press = true;
            }

            if (press) {
                Thread.sleep(WAIT_AFTER_PRESS);
                press = false;
            }

            telemetry.addData("Servo Pos", servo.getPosition());
            telemetry.addData("Dist", sonarRange.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
