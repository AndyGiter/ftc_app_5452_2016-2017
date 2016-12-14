package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by mlowery2 on 12/14/2016.
 *
 * To test the max speed the robot can turn 360 degrees. The reason for turning this much is the robot always turn less than this and the less the robot turns the more acurate the turn will be
 */

@TeleOp(name="Turn speed testing", group="testing")
public class TurnTest extends LinearBase {

    double turnSpeed = 0.1; // Default
    final int TURN_DEG = 360;
    final int WAIT_TIME = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        initalize(DcMotor.RunMode.RUN_USING_ENCODER, true);
        waitForStart();
        sleep(100);

            while(!gamepad1.a)
            {
                telemetry.addData("Welcome to the turn speed tester. Turning", TURN_DEG+"");
                telemetry.addData("Dpad up and down increases and decreases respectively", "");
                telemetry.addData("Current Speed", turnSpeed + "");
                telemetry.update();

                if(gamepad1.dpad_up)
                {
                    turnSpeed = Range.clip(turnSpeed + 0.1, 0, 1);
                    sleep(WAIT_TIME);
                }
                else if(gamepad1.dpad_down)
                {
                    turnSpeed = Range.clip(turnSpeed - 0.1, 0, 1);
                    sleep(WAIT_TIME);
                }
            }

        turn(turnSpeed, TURN_DEG);

        sleep(5000); // giving time to read the ending telemetry from the turn function

    }
}
